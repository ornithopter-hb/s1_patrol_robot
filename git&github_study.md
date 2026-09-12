version control, backup, collaboration
1. version control
	1. version?
	2. structure
		1. working tree
			1. the directory we created(the folder)
		2. stage
			1. the files stay to be made version
		3. repository
			1. the files staying in stage saved in version
	3. How does the system works? 
		1. edit files in the working tree
		2. tree -> stage : save in the stage that we want to make it version
		3. stage -> repo : commit to copies and paste the files in stage to repository
	4. git init 
		1. to use git resetting the directory 
		2. how?
			1. go to the specific directory we want to use
			2. in git bash, type 'git init'
		3. the directory is going to used as the repository
	5. git status
		1. untracked file : files that never been managed before
		2. changes to be committed : the files that needs to be committed to be seen on the repo
		3. nothing to commit : when everything is committed
		4. working tree clean : when everything is committed
		5. changes not staged for commit : files edited but not staged yet
	6. git add *file_name.extension*
		1. using to stage a file from working tree
	7. git commit
		1. making a version
		2. <ins>git commit --amend</ins> : edit commit messsage 
	8. git log
		1. to check the version(commit) is created as we intended
		2. commit hash : shows the id of the commit
		3. (HEAD -> master): shows this version is the latest one
		4. Author : shows who made this 
		5. Date : shows when this version is committed
		6. git log --stat : 
	9. git diff
		1. shows what is differed from the version we committed and now. 
	10. tracked and untracked
		1. tracked file : file that is committed before
		2. untracked file : new file, never been committed before
	11. .gitignore
		1. the file that I don't want to put in to the version
		2. to create the .gitignore
			1. use vim .gitignore to create the txt file
			2. write down the file's name and extension one file by one line.
			3. ex 
				1. hello.txt
				2. hello2.txt
				3. ...
	12. unmodified, modified, staged(all of these are tracked files)
		1. unmodified : (nothing to commit, working tree clean) -> nothing is edited
		2. modified : (changes not staged for commit) -> something is edited, but not staged yet
		3. staged : (changes to be committed) -> something is edited and staged
	13. git checkout -- filename.extension
		1. going back to the prior file that are not staged yet
	14. git reset HEAD filename.extension
		1. to unstage the staged file
	15. git reset HEAD^
		1. replacing the recent commit stepping down to the working tree
		2. --soft HEAD^
			1. moving back to before commit
		3. --mixed HEAD^
			1. moving back to recent commit and before staging
		4. -- hard HEAD^
			1. moving back to recent commit and before staging, and making the edited file to un-edited file
	16. git reset commit-hash 
		1. moving back to the wanted version
		2. type the commit-hash of the version that we want to go back to
		3. but it deletes the versions after that
	17. git revert
2. github
	1. git remote add origin <ins>address</ins>
		1. cd wanted folder
		2. commit
		3. git remote add origin <ins>address</ins>
	2. git remote -v 
		1. checking the connection between remote and origin
	3. git push -u origin master
		1. push local repo's branch to origin(github)
		2. -u means connecting local repo's branch to github's master branch
	4. git pull origin master
		1. getting files from the origin to local repo's master
	5. ssh
		1. ssh-keygen
			1. creates public and privacy keys
			2. add that key on the github's setting
			3. then we can easily log in to the github on git bash
3. collaboration
	1. git clone 
		1. git clone <ins>address</ins> <ins>directory</ins>
		2. get everything from the <ins>address</ins> to <ins>directory</ins>
	2. git fetch
		1. similar to the git pull, but it doesn't really pull the things on the github
		2. it gets new stuffs from the github and put it in to FETCH_HEAD
	3. git checkout master
		1. checking and comparing the github repo and the local repo
	4. git merge FETCH_HEAD
		1. merging new stuffs on the FETCH_HEAD to our local repo
		2. git pull = git fetch + git merge