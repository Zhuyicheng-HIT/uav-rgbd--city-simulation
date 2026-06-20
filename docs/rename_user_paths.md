# 用户名不是 zyc 时的路径替换

本项目建议下载者在 Ubuntu/WSL 中使用 `zyc` 作为用户名。这样可以直接复用已验证路径：

```text
/home/zyc/vision_sim_ws
/home/zyc/vision_avoid
/home/zyc/real_drone
```

如果已经创建了其他用户名，可以执行路径替换，但不推荐初学者使用。

## 先扫描

```bash
grep -R "/home/zyc" -n ~/vision_sim_ws ~/vision_avoid ~/real_drone
```

## 再替换

```bash
OLD_USER=zyc
NEW_USER="$USER"
grep -RIl "/home/${OLD_USER}" ~/vision_sim_ws ~/vision_avoid ~/real_drone \
  | xargs sed -i "s#/home/${OLD_USER}#/home/${NEW_USER}#g"
```

## 替换后检查

```bash
grep -R "/home/zyc" -n ~/vision_sim_ws ~/vision_avoid ~/real_drone
```

如果仍有输出，需要判断是否是文档示例，还是程序真实路径。

## 注意

- 替换前建议先提交 git 或备份。
- 不要替换第三方源码中的无关文本。
- 如果脚本使用 `~`，一般不需要替换。
- 如果 Gazebo 世界文件或模型文件中写了绝对路径，也需要一起检查。
