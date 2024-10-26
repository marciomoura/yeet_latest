# README

## Tool Requirements

### Clangd
1. Install the extension using [VSCode](https://github.com/clangd/vscode-clangd).
2. Under preferences, add `--query-driver=C:\msys64\mingw64\**\*` to enable Clangd to parse the system headers.

### Clang-tidy
1. Install [Clang-tidy](https://clang.llvm.org/extra/clang-tidy/).

### CppCheck
1. Install CppCheck and add it to the environmental path (e.g., `C:\Program Files\Cppcheck`).

## MSYS2

To compile the software, download the following packages through MSYS2:

```sh
pacman -S mingw-w64-x86_64-clang
pacman -S mingw-w64-x86_64-gdb
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-ccache
```

Add the respective paths to your local environment (for Windows development):

```sh
C:\msys64\usr\bin
C:\msys64\mingw64\bin
```

## Python Virtual Environment

1. Install Python.
2. In Windows, change the script policy to allow running scripts:

```sh
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

3. Run the following task in VSCode (Ctrl + Shift + P): `Initialize Python env`.