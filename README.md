# code-learning
上机准备代码
:dancer:
用http还是ssh取决于pull下来的方式，git remote -v查看当前远程仓库地址，移除http，git remote rm origin 再添加新的ssh方式的origin：

git remote add origin git@github.com:yychuyu/linux-system-programming.git

 git push --set-upstream origin master 设置一下上游要跟踪的分支
