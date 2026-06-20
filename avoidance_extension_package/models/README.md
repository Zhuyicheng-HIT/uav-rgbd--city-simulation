# models

模型文件不放入 GitHub。

## 仿真模型

推荐文件名：

```text
irreality.pt
irreality.engine
```

程序优先加载 TensorRT engine：

```text
irreality.engine
```

如果不存在，再加载：

```text
irreality.pt
```

## 实机模型

推荐文件名：

```text
people.pt
people.engine
```

## TensorRT 导出

导出 engine 需要较大的依赖安装和 GPU 环境，不建议作为基础安装的一部分。可单独执行：

```bash
yolo export model=irreality.pt format=engine imgsz=640 half=True
```

如果导出后 Python 环境异常，检查：

```bash
python3 -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"
```

必要时恢复：

```bash
pip install "numpy==1.26.4" -i https://pypi.tuna.tsinghua.edu.cn/simple
```

