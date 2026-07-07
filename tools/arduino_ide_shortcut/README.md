# Arduino IDE Shortcut Extraction

The original local shortcut was:

```text
C:\Users\hoffm\OneDrive\Desktop\Arduino IDE.lnk
```

Windows shortcut files are binary launcher files, not source-code files. This
shortcut did not contain an Arduino sketch path or any source code. It only
contained launcher metadata for opening Arduino IDE.

Extracted on 2026-07-07:

```text
Description: Arduino IDE
TargetPath: C:\Users\CodexSandboxOffline\AppData\Local\Programs\Arduino IDE\Arduino IDE.exe
Arguments:
WorkingDirectory: C:\Users\hoffm\AppData\Local\Programs\Arduino IDE
IconLocation: C:\Users\hoffm\AppData\Local\Programs\Arduino IDE\Arduino IDE.exe,0
```

For the normal user account, the equivalent command is expected to be:

```text
"C:\Users\hoffm\AppData\Local\Programs\Arduino IDE\Arduino IDE.exe"
```

No `.ino` file or Arduino project was embedded in the shortcut.
