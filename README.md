# sb_new_tor

机器人运控相关工程，当前仓库包含：

- `WK_ZY/`：ROS2 与电机通信相关代码
- `yunkongzhongyi/`：EtherCAT 主站与示例工程
- `probe/`：延时探针与辅助监测代码

## 仓库说明

当前 `.gitignore` 已排除常见 ROS 构建产物、本机 IDE 目录和运行日志目录，适合直接作为 GitHub 仓库基础版本。


## git指令
``` bash
git tag //这会列出仓库中所有的 tags
git branch -l //查看当前分支
git checkout v1.2 //切换分支
git status  //查看当前状态
git pull origin main //拉取并合并远程仓库的代码
git stash //暂存当前未提交的代码
git stash pop //恢复暂存的代码
```

推送流程
```bash
git add .
git commit -m "提交信息"
git push -u origin main //推送代码到远程仓库

```

