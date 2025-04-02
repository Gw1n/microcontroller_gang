Git commands
 Starting commands
 1. 
git config --global user.name "[name]"
 - setting dev name
 2. 
git config --global user.email "[email]"
 - setting dev email
 3. 
cd [reference]
 - reference to the folder of your future repository
 4. 
git init
 - setting the local repository to work with
 Essential work
 1. 
git add .
 - adding all files to index (to commit them in the future)
 2. 
git status
 - cheking status of the repository
 3. 
git commit -m "[comment for new version]"
 - saving (commiting) current version of the
 project in the local repository
 4. 
git log
 - info about latest commits (also possible to use flag 
more info about commits)
 5. -p
 at the end, so git shows
 git show [hash]
 - shows all changes in mentioned by hash commit
 6. 
git revert HEAD
 - rolls back latest commit, creating the new one. Instead of HEAD
 possible to mention hash of the commit
 7. 
git branch [new_branch_name]
 - creating new branch
 8. 
git checkout [new_branch_name]
 - switching to new branch
 9. 
git branch
 - shows all branches in repository, current branch would be mentioned with "*"
 Operating with GitHub
 1. 
git remote add [name] [url]
 - setting remote repository (on GitHub in our case)
 2. 
git remote show [name]
 - shows info about remote repository
 3. 
git push [name] [branch]
 - sending code to the remote repository
 4. 
git pull
 - downloading current code from remote repository (works only with current
 branch)
 Folders naming
 For instance: 1.4.1 actually means 1 - lab number, 4 - task number, 1 - relevant version
