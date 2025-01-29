# YEET Platform 🚀

Yo! Welcome to YEET - Your Power Electronics Engineering Software Platform! This platform is like your bestie when it comes to working with power electronics products. Fr fr, we're talking about next-level data modeling and closed-loop testing that'll make your power electronics development smooth af.

## What's the Tea? 🫖

YEET is built different - it's not just another engineering tool. Here's what makes it slap:

- Dropping sick data modeling capabilities that help you understand your power electronics inside out
- Serving closed-loop testing libraries that hit different when you need to validate your designs
- Coming soon: Microcontroller support that's gonna be straight bussin'
- RTOS integration in the works that'll have your real-time applications looking clean

No cap, we're working on making this the most comprehensive platform for power electronics development. The roadmap's looking kinda spicy with upcoming support for various microcontrollers and RTOS - stay tuned fam!

## Getting Started (aka The Tutorial Level) 🎮

### Tools You'll Need (Basic Loadout)

First things first, let's get your setup looking right:

#### Clangd (Your Code BFF)
1. Pull up [VSCode](https://github.com/clangd/vscode-clangd) and grab that Clangd extension
2. In preferences, drop this line: `--query-driver=C:\msys64\mingw64\**\*` (helps Clangd understand those system headers)

#### Clang-tidy (The Code Style Police)
1. Get [Clang-tidy](https://clang.llvm.org/extra/clang-tidy/) - it's like spell check but for C++

#### CppCheck (The Vibe Checker)
1. Install CppCheck and add it to your environmental path (slide it into `C:\Program Files\Cppcheck`)

### MSYS2 Setup (The Power-Up) 💪

Drop these commands in MSYS2 to get the essential packages:

```sh
pacman -S mingw-w64-x86_64-clang
pacman -S mingw-w64-x86_64-gdb
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-ccache
pacman -S mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-lcov
pacman -S mingw-w64-x86_64-perl
```

For the Windows squad, add these paths to your local environment:
```sh
C:\msys64\usr\bin
C:\msys64\mingw64\bin
```

### Python Virtual Environment (The Safe Space) 🐍

1. Grab Python (duh!)
2. If you're on Windows, run this in PowerShell to keep things secure:
```sh
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```
3. Hit up VSCode with Ctrl + Shift + P and run `Initialize Python env`

And that's the tea! You're now ready to start yeeting your power electronics projects to the next level! If you're stuck or something's not giving what it's supposed to give, drop an issue in our repo. We got you! 💅✨

No cap, this project is still in development, so expect some features to be cooking. But trust, when they drop, they're gonna be fire! 🔥