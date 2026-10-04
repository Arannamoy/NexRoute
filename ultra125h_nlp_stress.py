"""
Ultra 5 125H, 10 GB RAM — one file, 5 hour NLP stress.

Run:
    python ultra125h_nlp_stress.py

Writes nlp_stress.log next to this file. Terminal shows the same lines.

DirectML training lands on Arc (7 Xe-core). NPU flood uses OpenVINO.
A line with backend=...NPU means the NPU is actually working.
directml-ep:likely-igpu means the NPU was missed.

10 GB profile is baked in. Override with env DURATION_HOURS if you want a short test.
"""

from __future__ import annotations

import json
import math
import os
import sys
import time
from multiprocessing import Process, Queue
from pathlib import Path

ROOT = Path(__file__).resolve().parent
LOG_PATH = ROOT / "nlp_stress.log"

HOURS = float(os.environ.get("DURATION_HOURS", "5"))
BATCH = 2
SEQ = 128
D_MODEL = 256
N_LAYERS = 4
N_HEADS = 4
VOCAB = 8000
CPU_WORKERS = 4
NPU_BATCH = 2
NPU_SEQ = 64
NPU_D = 128
NPU_LAYERS = 2
NPU_HEADS = 4
NPU_VOCAB = 4000


def _emit(q: Queue, row: dict) -> None:
    row["t"] = time.strftime("%H:%M:%S")
    q.put(row)


def train_worker(q: Queue, deadline: float) -> None:
    try:
        import torch
        import torch.nn as nn
        import torch.nn.functional as F
        import torch_directml
    except ImportError as exc:
        _emit(q, {"src": "train", "event": "missing", "err": str(exc), "fix": "pip install torch-directml"})
        return
    if not torch_directml.is_available():
        _emit(q, {"src": "train", "event": "no_directml"})
        return

    device = torch_directml.device(0)
    name = torch_directml.device_name(0)

    class Attn(nn.Module):
        def __init__(self):
            super().__init__()
            self.n_heads = N_HEADS
            self.hd = D_MODEL // N_HEADS
            self.qkv = nn.Linear(D_MODEL, 3 * D_MODEL)
            self.proj = nn.Linear(D_MODEL, D_MODEL)

        def forward(self, x):
            b, t, c = x.shape
            qkv = self.qkv(x).reshape(b, t, 3, self.n_heads, self.hd).permute(2, 0, 3, 1, 4)
            qq, k, v = qkv[0], qkv[1], qkv[2]
            att = (qq @ k.transpose(-2, -1)) * (1.0 / math.sqrt(self.hd))
            mask = torch.triu(torch.ones(t, t, device=x.device, dtype=torch.bool), diagonal=1)
            att = torch.softmax(att.masked_fill(mask, float("-inf")), dim=-1)
            y = (att @ v).transpose(1, 2).reshape(b, t, c)
            return self.proj(y)

    class Block(nn.Module):
        def __init__(self):
            super().__init__()
            self.ln1 = nn.LayerNorm(D_MODEL)
            self.attn = Attn()
            self.ln2 = nn.LayerNorm(D_MODEL)
            self.mlp = nn.Sequential(nn.Linear(D_MODEL, 4 * D_MODEL), nn.GELU(), nn.Linear(4 * D_MODEL, D_MODEL))

        def forward(self, x):
            x = x + self.attn(self.ln1(x))
            return x + self.mlp(self.ln2(x))

    class LM(nn.Module):
        def __init__(self):
            super().__init__()
            self.tok = nn.Embedding(VOCAB, D_MODEL)
            self.pos = nn.Embedding(SEQ, D_MODEL)
            self.blocks = nn.ModuleList(Block() for _ in range(N_LAYERS))
            self.ln = nn.LayerNorm(D_MODEL)
            self.head = nn.Linear(D_MODEL, VOCAB, bias=False)

        def forward(self, idx, targets):
            pos = torch.arange(idx.size(1), device=idx.device)
            x = self.tok(idx) + self.pos(pos)
            for block in self.blocks:
                x = block(x)
            logits = self.head(self.ln(x))
            return F.cross_entropy(logits.reshape(-1, VOCAB), targets.reshape(-1))

    model = LM().to(device)
    optim = torch.optim.AdamW(model.parameters(), lr=3e-4, betas=(0.9, 0.95))
    params = sum(p.numel() for p in model.parameters())
    _emit(q, {"src": "train", "event": "start", "device": str(device), "name": name, "params_m": round(params / 1e6, 2)})
    step = 0
    last = time.time()
    model.train()
    while time.time() < deadline:
        idx = torch.randint(0, VOCAB, (BATCH, SEQ), device=device)
        optim.zero_grad(set_to_none=True)
        loss = model(idx[:, :-1], idx[:, 1:])
        loss.backward()
        optim.step()
        step += 1
        if step % 20 == 0:
            now = time.time()
            loss_v = float(loss.detach().to("cpu"))
            _emit(q, {
                "src": "train",
                "step": step,
                "loss": round(loss_v, 4),
                "tok_s": round(20 * BATCH * (SEQ - 1) / max(now - last, 1e-6), 1),
            })
            last = now
    _emit(q, {"src": "train", "event": "done", "step": step})


def _build_onnx(path: Path) -> None:
    import numpy as np
    import onnx
    from onnx import TensorProto, helper, numpy_helper

    d, h, seq, b = NPU_D, NPU_HEADS, NPU_SEQ, NPU_BATCH
    hd = d // h
    nodes, inits = [], []

    def add_w(name, shape):
        inits.append(numpy_helper.from_array(np.random.randn(*shape).astype(np.float32) * 0.02, name))

    add_w("emb", (NPU_VOCAB, d))
    nodes.append(helper.make_node("Gather", ["emb", "input_ids"], ["x0"], axis=0))
    x = "x0"
    for i in range(NPU_LAYERS):
        add_w(f"w{i}", (d, d))
        add_w(f"w1{i}", (d, 4 * d))
        add_w(f"w2{i}", (4 * d, d))
        nodes.append(helper.make_node("MatMul", [x, f"w{i}"], [f"p{i}"]))
        nodes.append(helper.make_node("Add", [x, f"p{i}"], [f"r{i}"]))
        nodes.append(helper.make_node("MatMul", [f"r{i}", f"w1{i}"], [f"h1{i}"]))
        nodes.append(helper.make_node("Relu", [f"h1{i}"], [f"h1r{i}"]))
        nodes.append(helper.make_node("MatMul", [f"h1r{i}", f"w2{i}"], [f"h2{i}"]))
        nodes.append(helper.make_node("Add", [f"r{i}", f"h2{i}"], [f"x{i+1}"]))
        x = f"x{i+1}"
    nodes.append(helper.make_node("ReduceMean", [x], ["pooled"], axes=[1], keepdims=0))
    add_w("cls", (d, 4))
    nodes.append(helper.make_node("MatMul", ["pooled", "cls"], ["logits"]))
    graph = helper.make_graph(
        nodes,
        "npu_nlp",
        [helper.make_tensor_value_info("input_ids", TensorProto.INT64, [b, seq])],
        [helper.make_tensor_value_info("logits", TensorProto.FLOAT, [b, 4])],
        inits,
    )
    model = helper.make_model(graph, opset_imports=[helper.make_opsetid("", 17)])
    model.ir_version = 8
    onnx.save(model, path)


def npu_worker(q: Queue, deadline: float) -> None:
    import numpy as np

    model_path = ROOT / "npu_encoder.onnx"
    try:
        _build_onnx(model_path)
    except ImportError as exc:
        _emit(q, {"src": "npu", "event": "missing", "err": str(exc), "fix": "pip install onnx onnxruntime"})
        return

    handle = None
    backend = None
    errors = []
    try:
        import onnxruntime as ort

        so = ort.SessionOptions()
        sess = ort.InferenceSession(
            str(model_path),
            sess_options=so,
            providers=[("OpenVINOExecutionProvider", {"device_type": "NPU", "precision": "ACCURACY"})],
        )
        if "OpenVINOExecutionProvider" not in sess.get_providers():
            raise RuntimeError(sess.get_providers())
        handle, backend = sess, "onnxruntime-openvino:NPU"
    except Exception as exc:  # noqa: BLE001
        errors.append(f"ort-ov: {exc}")
    if handle is None:
        try:
            import openvino as ov

            core = ov.Core()
            if "NPU" not in core.available_devices:
                raise RuntimeError(f"devices={core.available_devices}")
            handle, backend = core.compile_model(str(model_path), "NPU"), "openvino:NPU"
        except Exception as exc:  # noqa: BLE001
            errors.append(f"ov: {exc}")
    if handle is None:
        try:
            import onnxruntime as ort

            sess = ort.InferenceSession(str(model_path), providers=["DmlExecutionProvider", "CPUExecutionProvider"])
            if "DmlExecutionProvider" not in sess.get_providers():
                raise RuntimeError(sess.get_providers())
            handle, backend = sess, "directml-ep:likely-igpu"
        except Exception as exc:  # noqa: BLE001
            errors.append(f"dml: {exc}")
            _emit(q, {"src": "npu", "event": "bind_fail", "err": " | ".join(errors)})
            return

    npu_ok = backend.endswith("NPU")
    _emit(q, {"src": "npu", "event": "bound", "backend": backend, "npu_working": npu_ok, "tried": errors})
    ids = np.random.randint(0, NPU_VOCAB, size=(NPU_BATCH, NPU_SEQ), dtype=np.int64)
    step = 0
    last = time.time()
    while time.time() < deadline:
        if backend.startswith("openvino:"):
            handle.create_infer_request().infer({"input_ids": ids})
        else:
            out = handle.run(None, {"input_ids": ids})
            ids[0, 0] = (int(ids[0, 0]) + int(out[0].flat[0]) + 1) % NPU_VOCAB
        step += 1
        if step % 40 == 0:
            now = time.time()
            _emit(q, {
                "src": "npu",
                "step": step,
                "backend": backend,
                "npu_working": npu_ok,
                "infer_s": round(40 / max(now - last, 1e-6), 2),
                "tok_s": round(40 * NPU_BATCH * NPU_SEQ / max(now - last, 1e-6), 1),
            })
            last = now
    _emit(q, {"src": "npu", "event": "done", "step": step, "backend": backend})


def cpu_worker(q: Queue, wid: int, deadline: float) -> None:
    import numpy as np

    rng = np.random.default_rng(wid + 3)
    a = rng.standard_normal((128, 384), dtype=np.float32)
    b = rng.standard_normal((384, 384), dtype=np.float32)
    text = rng.integers(0, 8000, size=(4096,), dtype=np.int32)
    step = 0
    last = time.time()
    while time.time() < deadline:
        grams = (text[:-2] * 1315423911 + text[1:-1] * 2654435761 + text[2:]) % 8000
        text[: len(grams)] = grams
        a = (a @ b)[:128] * 0.999 + 0.001
        step += 1
        if step % 15 == 0 and wid == 0:
            now = time.time()
            _emit(q, {"src": "cpu", "worker0_step": step, "gemm_s": round(15 / max(now - last, 1e-6), 2)})
            last = now
    if wid == 0:
        _emit(q, {"src": "cpu", "event": "done", "step": step})


def monitor_worker(q: Queue, deadline: float) -> None:
    try:
        import psutil
    except ImportError:
        _emit(q, {"src": "mon", "event": "missing", "fix": "pip install psutil"})
        return
    psutil.cpu_percent(interval=None)
    while time.time() < deadline:
        per = psutil.cpu_percent(interval=5, percpu=True)
        _emit(q, {
            "src": "mon",
            "cpu_avg": round(sum(per) / max(len(per), 1), 1),
            "cpu_max": round(max(per) if per else 0, 1),
            "ram_pct": round(psutil.virtual_memory().percent, 1),
        })


def main() -> None:
    deadline = time.time() + HOURS * 3600
    q: Queue = Queue()
    header = {
        "src": "main",
        "event": "start",
        "chip": "Ultra 5 125H profile",
        "hours": HOURS,
        "batch": BATCH,
        "seq": SEQ,
        "d_model": D_MODEL,
        "layers": N_LAYERS,
        "log": str(LOG_PATH),
    }
    LOG_PATH.write_text(json.dumps(header) + "\n", encoding="utf-8")
    print(json.dumps(header), flush=True)

    procs = [
        Process(target=train_worker, args=(q, deadline), name="train"),
        Process(target=npu_worker, args=(q, deadline), name="npu"),
        Process(target=monitor_worker, args=(q, deadline), name="mon"),
    ]
    procs += [Process(target=cpu_worker, args=(q, i, deadline), name=f"cpu{i}") for i in range(CPU_WORKERS)]
    for p in procs:
        p.start()

    alive = True
    with LOG_PATH.open("a", encoding="utf-8") as fh:
        while alive:
            try:
                row = q.get(timeout=2)
            except Exception:
                row = None
            if row:
                line = json.dumps(row)
                fh.write(line + "\n")
                fh.flush()
                print(line, flush=True)
            alive = any(p.is_alive() for p in procs) or not q.empty()
            if time.time() > deadline + 30:
                break
    for p in procs:
        p.join(timeout=5)
        if p.is_alive():
            p.terminate()
    end = {"src": "main", "event": "done", "log": str(LOG_PATH)}
    with LOG_PATH.open("a", encoding="utf-8") as fh:
        fh.write(json.dumps(end) + "\n")
    print(json.dumps(end), flush=True)


if __name__ == "__main__":
    main()
