#!/bin/bash

# 定义路径
ONNX_MODEL="/home/delphine/rm/tensorrt10_detect/models/yolo26.onnx"
OUTPUT_ENGINE="/home/delphine/rm/tensorrt10_detect/models/yolo26.engine"
CONVERT_TOOL="/home/delphine/rm/tensorrt10_detect/build/export"

# 检查转换工具是否存在
if [ ! -f "$CONVERT_TOOL" ]; then
    echo "错误：转换工具不存在，请先编译项目"
    echo "请运行：cd /home/delphine/rm/tensorrt10_detect/build && cmake .. && make"
    exit 1
fi

# 检查 ONNX 文件是否存在
if [ ! -f "$ONNX_MODEL" ]; then
    echo "错误：ONNX 文件不存在：$ONNX_MODEL"
    exit 1
fi

# 运行转换工具
echo "正在将 $ONNX_MODEL 转换为 TensorRT 引擎..."
echo "输出文件：$OUTPUT_ENGINE"

# 设置 TensorRT 库路径
export LD_LIBRARY_PATH=/opt/TensorRT/TensorRT-10.2.0.19/lib:$LD_LIBRARY_PATH

# 执行转换
"$CONVERT_TOOL" "$ONNX_MODEL" "$OUTPUT_ENGINE"

# 检查转换结果
if [ $? -eq 0 ]; then
    echo "转换成功！"
    if [ -f "$OUTPUT_ENGINE" ]; then
        echo "引擎文件已生成：$OUTPUT_ENGINE"
        echo "引擎大小：$(du -h "$OUTPUT_ENGINE" | cut -f1)"
    else
        echo "转换命令执行成功，但未生成引擎文件"
    fi
else
    echo "转换失败，请检查错误信息"
    exit 1
fi
