# monash_main
Main module for autonomous drone project

# Clang How-To
Install the clang ecosystem:
```
wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key|sudo apt-key add -
sudo add-apt-repository 'deb http://apt.llvm.org/bionic/ llvm-toolchain-bionic main'
sudo apt update
sudo apt install -y clangd clang-format clang-tidy
```

Install the **vscode-clangd** extension, and don't forget to disable your current linter.

In order for clang to correctly parse your included files, it needs a list of compile commands that are used to build the project. The easiest way to obtain this to use the standard cmake modules system from here, which will handle it automatically. Alternatively: 
```
catkin config --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```
before you `catkin build`, and then create a symlink in `your_node/` that points to `catkin_ws/build/your_node/compile_commands.json`.

eg, whilst in `.../YOUR_NODE/` :

```
ln -s ${HOME}/autonomous-drone/catkin_ws/build/YOUR_NODE/compile_commands.json .
ln -s ${HOME}/autonomous-drone/catkin_ws/src/monash_main/.clang-format .
ln -s ${HOME}/autonomous-drone/catkin_ws/src/monash_main/.clang-tidy .
```

Create symlinks in `your_node/` that point to the `.clang-tidy` and `.clang-format` files here.

The easiest way to do this is to simply run the script: ```scripts/clang_links.sh```

Don't forget to create a global gitignore file to ignore these new files in all your submodules
Create a '.gitignore_global' file to the base level
Don't forget to add `compile_commands.json`,`clangd`,`.clang-format`,`.clang-tidy` to your file

```
git config --global core.excludesfile ~/.gitignore_global
```

**clang-tidy** will run better if you add the following commands the settings of the vscode extension:

 - --suggest-missing-includes
 - --background-index
 - -j=4
