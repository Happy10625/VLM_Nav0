#!/usr/bin/env bash

# Keep a generated terminal open after its launch command exits.
set +e
script="$1"
shift
bash "$script" "$@"
status=$?

echo
if (( status == 0 )); then
  echo "进程已结束（exit=${status}）。"
else
  echo "启动失败（exit=${status}），请检查上方错误。"
fi
read -r -p "按 Enter 关闭此终端……" _
exit "$status"
