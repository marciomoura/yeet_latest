# README 

# Tool requirements:

## Clangd:
1. Install the extension using vscode (https://github.com/clangd/vscode-clangd);
2. Under preferences add `--query-driver=C:\msys64\mingw64\**\*` to enable clangd to parse the system headers;

## Clang-tidy:
1. Install clang-tidy (https://clang.llvm.org/extra/clang-tidy/)

## CppCheck:
1. Install cppcheck and add it to the environmental path (e.g. C:\Program Files\Cppcheck)

# MSYS2

To compile the software, the following packages can be downloaded through MSYS2:

pacman -S mingw-w64-x86_64-clang
pacman -S mingw-w64-x86_64-gdb 
pacman -S mingw-w64-x86_64-cmake 
pacman -S mingw-w64-x86_64-ccache

You can add the respective paths to your local environment (for Windows development):
C:\msys64\usr\bin
C:\msys64\mingw64\bin


# Python Virtual Environment

First, install python

In windows you will need to change the script policy scripts to run on

 Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser

Then run the following task (in vscode through ctrl + shift + p): run task `Initialize Python env`

