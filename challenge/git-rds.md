# Your Master tool: sharing and versioning 

First tool needed by Developper teams is sharing solution (i.e. a solution allowing each teammate to contribute over common files).
The corollary need (most important than sharing) is to guarantee that no one would be capable to destroy the works of the others, so **VERSIONING** (i.e. a solution allowing each teammate to secure its contributions with the possibility to check out any versions in the contribution timeline).

**Git** is the most popular versioning solution nowadays.
It particularity is to distribute the versions potentially over all the contributors. More on [Wikipedia](https://en.wikipedia.org/wiki/Git).

Then a software solution (*gitlab* for instance) deployed on a server will permit a team to share their project (i.e. common versions).
There is a large number of those git-web-services as *github.com* for the most popular (gitlab.com, bitbucket.org, gvipers.imt-lille-douai.fr).

Another advantage of git is that the solution is available on TheConstruct environment.

## Initialize a git repository

First you have to create a *repo* (a shared directory or repository) on a git-web-service.

1. Connect to [GitHub](https://github.com) platform.
2. Create an account.

Then only one member of the team has to perform the next instructions.

3. Create a new repository
   - Initialize a `README.md` with a simple tittle (cf. [Markdown syntax](https://fr.wikipedia.org/wiki/Markdown))
4. Invite your teammate with the higher possible status.
5. Invite your professors (the organization [ceri-num](https://github.com/ceri-num))

## Sharing

It will be now possible to clone this *repo* on a fresh TheConstruct project.

1. Open a fresh ROSject on Kinetic with no template.
2. Clone the distant *repo* on a dedicated directory (`project_ws` for instance).

```bash
git clone MyHTTPsURL project_ws
```

3. Play with your teammate to merge modification in the `README.md` file.

From that point, the `README.md` file appears on your `project_ws` directory in the editor.
By editing this file, you create **localy** a new version. 
Try `git status` to visualize the modification.
Then you can save this new version with `git commit -am 'new README'`.
The option **a** and **m** are respectively for **all modification** and 
**message**.

Git will ask you for your identity. 
You have to fill the appropriate command before to run again the `git commit` command.

The new version is only local.
You can share it by pushing it on the server. 
First, you have to `git pull` in order to get any new version on the server.
Second (if there is no merge conflict ;) ), you can `git push`.

Finally, you can find on the internet, plenty of tutorial like [this one](https://opensource.com/article/20/4/git-merge-conflict) to deal with merging conflicts.

## Initialize a proper catkin project

(Only one member of the team has to perform the next instructions)

As you already learn, a catkin project can be initialized with a `src` directory and the command `catkin_make`.
Catkin would generate configuration and construction files easily highlighted with `git status`.

```bash
mkdir src
catkin_make
git status
```

There is no need to version the construction files so we ask git to ignore it with a `.gitignore` file.

```bash
echo "
.catkin_workspace
build/
devel/
" > .gitignore
git status
```

In contrary, it is necessary ask git to track `.gitignore` and the current file in `src` (and to share this initialization)

```bash
git add .gitignore src
git status
# On branch master
# Your branch is up-to-date with 'origin/master'.
# Changes to be committed:
#  (use "git reset HEAD <file>..." to unstage)
#        new file:   .gitignore
#        new file:   src/CMakeLists.txt
git commit -am 'Initialize a catkin workspace'
git status
```

Again `git status` will state that your working directory is clean, but with the recommendation to push your modifications on the server (the origin remote repository).

```bash
git pull
git push
```

You can verify on your server that two new files `.gitignore` and `src/CMakeLists.txt` appear and you can ask your teammate to update their workspace (`git commit -am tmp && git pull`).
