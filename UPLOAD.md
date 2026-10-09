# 上传 GitHub

本目录已经包含选定的五个模块（含 `PhybotSoftware_c2 arm`）、整理后的仓库首页和原 README 文档。原工程不受影响。

当前目录已初始化 Git，并已有首次提交。第 1～4 节留作首次建仓参考；追加机器人端时直接使用下面的步骤，不需要再次初始化或添加 `origin`。

## 追加机器人端

机器人端上传副本已经准备好，详见 [整理说明](docs/robotside.md)。在终端中执行：

```bash
cd '/home/chaochao/桌面/双视觉版本/github-upload'
git add 'PhybotSoftware_c2 arm' README.md UPLOAD.md .gitignore docs/robotside.md
git diff --cached --stat
```

确认暂存内容后提交和推送：

```bash
git commit -m 'Add Phybot robot-side control software'
git push -u origin main
```

目录名带空格，命令中的引号需要保留。整理操作只准备工作区文件，没有代为提交或推送。

## 1. 创建远程空仓库

打开 <https://github.com/new>，选择你的账户，仓库名可用 `dual-vision-navigation`，按需要选择 Private 或 Public。

不要在 GitHub 创建时勾选 README、.gitignore 或 License；本地已经准备了内容，空仓库便于首次推送。创建后复制 HTTPS 仓库地址。

## 2. 在这个上传目录初始化 Git

下面是首次初始化的参考命令；已经有 `.git` 的当前上传目录无需重复执行。

```bash
cd '/home/chaochao/桌面/双视觉版本/github-upload'
git init -b main
```

如果这台机器尚未设置提交身份，在该目录设置当前仓库使用的身份。将示例替换为自己的信息；邮箱也可以使用 GitHub 提供的 noreply 地址。

```bash
git config user.name '你的名字'
git config user.email '你的提交邮箱'
```

## 3. 查看并提交

```bash
git add .
git status --short
git diff --cached --stat
```

确认文件列表后执行：

```bash
git commit -m 'Initial import of dual vision navigation project'
```

请在 `github-upload` 中操作。上一级工程目录还包含其他版本和多个大型压缩包，不属于此次上传范围。

## 4. 关联远程仓库并上传

把下面 URL 中的 `YOUR_USERNAME` 替换为自己的 GitHub 用户名；如果仓库名不同，也一并替换。

```bash
git remote add origin https://github.com/YOUR_USERNAME/dual-vision-navigation.git
git remote -v
git push -u origin main
```

HTTPS 验证时使用已配置的凭据管理器或 GitHub Personal Access Token；不要把 Token 写入仓库 URL、源码或文档。若终端要求密码，GitHub 账户登录密码不能代替 Token。已经配置 SSH 的用户也可使用 `git@github.com:YOUR_USERNAME/dual-vision-navigation.git` 作为远程地址。

若提示 `remote origin already exists`，先运行 `git remote -v` 确认当前地址，再根据实际需要执行 `git remote set-url origin 新地址`。

若提示远端已有提交而拒绝推送，先检查远端内容；不要直接强制覆盖。上述步骤针对新建空仓库。

## 资源与大文件

本次整理保留 ONNX 和仿真网格，按普通 Git 仓库准备，尚未启用 Git LFS。两个最大的文件为 `car.obj` 和 `audi r8.obj`，各约 73 MiB；导出时没有超过 100 MiB 的普通文件。它们是场景网格，不能通过忽略所有 `*.obj` 来清理。

后续若添加更大的模型或数据，先核对 GitHub 的当前文件限制，再选择 Git LFS 或外部下载方式。LFS 属性应在首次添加相应文件前配置；添加规则不会自动清除已有提交中的大文件。

构建目录、wheel、地图和压缩包仍在原工程中；如果需要完整部署备份，应另外保存这些资源。源码仓库不能替代这些离线部署材料。

原 cuVSLAM 已存在的 53 个 LFS 指针文件另见 `docs/lfs-placeholders.txt`。当前副本保留其原样，尚未获取对应的图片实体。

## 后续更新

将修改同步到本上传目录后，在这里执行：

```bash
git status
git add .
git diff --cached --stat
git commit -m 'Describe the changes'
git push
```

这是独立副本，在原目录修改文件不会自动更新这里的内容。

## 官方说明

- [将本地代码添加到 GitHub](https://docs.github.com/en/migrations/importing-source-code/using-the-command-line-to-import-source-code/adding-locally-hosted-code-to-github)
- [GitHub 大文件说明](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-large-files-on-github)
- [GitHub 身份验证方式](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/about-authentication-to-github)

整理日期：2026-10-09。本次环境未能联网读取这些官方页面；平台限制及界面选项以 GitHub 当前说明为准。
