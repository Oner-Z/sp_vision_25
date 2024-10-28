#!/bin/bash

# 检查是否在一个 Git 仓库中
if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
    echo "请在一个 Git 仓库中运行此脚本。"
    exit 1
fi

# 找到更改过的文件和新增的未被追踪的文件
changed_files=$(git diff --name-only)
untracked_files=$(git ls-files --others --exclude-standard)

# 合并文件列表
all_files="$changed_files
$untracked_files"

# 检查是否有文件需要压缩
if [ -z "$all_files" ]; then
    echo "没有更改过的文件或新增的未被追踪的文件。"
    exit 0
fi

# 创建压缩包文件名
zip_file="changed_files.zip"

# 创建压缩包
zip -r "../$zip_file" $all_files

echo "已将更改过的文件和新增的未被追踪的文件压缩到 $zip_file。"
