---
vertion: tcs (TheConstructSim)
---
# Challenge Kick-Off


The challenge aims to put learners in the development of a micro robotic project.
In the end, both the execution of the proposed solution and the source code with documentation would be evaluated.

The main objectives of the project consist of:

- Control a robot in a cluttered environment
- Map a static environment
- Detect all the **Nuka-Cola**
- Estimate the position of the **Nuka-Cola** in the map
- Optimaze the exploration strategy

<https://www.myminifactory.com/object/3d-print-fallout-nuka-cola-15616>

![](https://cdn.myminifactory.com/assets/object-assets/579fca2a374fc/images/720X720-7a4418213f3ce580bb21f641c36650bd5eb8cdb3.jpg)


## Agile software development

Agile software development aims to break with traditional project management by preferring:

- **Individuals and Interactions** over processes and tools
- **Working Software** over comprehensive documentation
- **Customer Collaboration** over contract negotiation
- **Responding to Change** over following a plan

The main feature of Agile Software Development consists of iterative development by moving forward incrementally
and by delivering operational versions frequently.

More on [WikiPedia](https://en.wikipedia.org/wiki/Agile_software_development).


## Master tool: sharing and versioning 

git + gitlab or github or bitbucket.

README.md

*2* branches (master and dev)

## Go go go...

### Create a group


## Initialise a git repository

1. Create a repository on a git web service
   - Connect to GitHub platform
   - Create a new repository (by one member of the team)
   - Initialize a `README.md` with a simple tittle (cf. [markdown syntax]())

Cf. 

Clone this repo on a fresh TheConstruct project

Open a fresh ROSject (Kinetic - no template)

Clone the distant repo on a temporal diretory, then copy its content in the root of the our ROSject

```bash
git clone MyHTTPsURL project_ws
```

mkdir src
catkin_make

```bash
git status
```
```
Untracked files:
  (use "git add <file>..." to include in what will be committed)

        .catkin_workspace
        build/
        devel/
        src/
```

Ignore construction files.

.gitignore

```
.catkin_workspace
build/
devel/
```

```bash
git status
```
```
Untracked files:
  (use "git add <file>..." to include in what will be committed)

        .gitignore
        src/
```

```bash
git add .gitignore src
git status
```

```
On branch master
Your branch is up-to-date with 'origin/master'.
Changes to be committed:
  (use "git reset HEAD <file>..." to unstage)

        new file:   .gitignore
        new file:   src/CMakeLists.txt
```

```bash
git commit -am 'Initialise a catkin workspace'
```

Git will ask you for your identity. 
You have to fill the appropriate command before to run again the `git commit` command.

Again `git status` will state that your working directory is clean, but with the recommandation to push your modifications on the server (the origine remote repository).

So first get the new version from origine if any.

```bash
git pull
```

Normally it is already up-to-date. And finally push your version on the server.

```bash
git push
```

Now you can verrify on your server that tow new files `.gitignore` and `src/CMakeLists.txt` appear.


## Share your git repository




A component development is generally composed by **5** tasks to perform in a specific order:

- Defining and implementing the test procedures that would validate the efficiency of the component.
It is a good practice (named [test-driven development)[https://fr.wikipedia.org/wiki/Test_driven_development]) to set up the test before to start any development.
- Developing the component in a way the component passes all the tests.
- Testing the overall solution. Redo the test of all the already released components to validate that the development of the new component has not broken anything.
- Updating the documentation.
- Releasing the actual version (if all tests are green). In our case, it consists of merging the *dev* branch into the *master* branch, in order to always keep a working version on the *master* branch.

So, you just have to select a first component to develop, put it in the *doing* areas of your *team board*, and let go...










## Versioning and sharing

git + gitlab or github or bitbucket.

README.md

*2* branches (master and dev)


## Go go go...

A component development is generally composed by **5** tasks to perform in a specific order:

- Defining and implementing the test procedures that would validate the efficiency of the component.
It is a good practice (named [test-driven development)[https://fr.wikipedia.org/wiki/Test_driven_development]) to set up the test before to start any development.
- Developing the component in a way the component passes all the tests.
- Testing the overall solution. Redo the test of all the already released components to validate that the development of the new component has not broken anything.
- Updating the documentation.
- Releasing the actual version (if all tests are green). In our case, it consists of merging the *dev* branch into the *master* branch, in order to always keep a working version on the *master* branch.

So, you just have to select a first component to develop, put it in the *doing* areas of your *team board*, and let go...
