# Preferences, recent files and updates

## Preferences

*File → Preferences* (`Ctrl+,`) holds the settings that apply to every project. Each change
is saved at once and is kept for the next session.

| Setting | What it does |
| --- | --- |
| Units | The units new projects start in. *As the last project* keeps the units of the project before. |
| Autosave | How old unsaved changes get (in seconds) before a recovery copy is written; off writes none. See [autosave and recovery](#autosave-and-recovery). |
| Material, post processor, machine | What *Set Up the Selected Part* on the CAM panel starts a job with. A lathe job takes the material, and the post when the lathe has it. |
| Updates | Look for a newer release when SI CDM starts, from the release feed your administrator gives you. |

The same settings can be typed:

```text
PREFERENCES                          list them, and the file they are kept in
PREFERENCES SET units in             mm, cm, m, in, ft, pt, or LAST
PREFERENCES SET autosave 120         seconds (5 to 3600), or OFF
PREFERENCES SET material steel       a CAM material, or DEFAULT
PREFERENCES SET post haas            fanuc, haas, grbl, linuxcnc, or DEFAULT
PREFERENCES SET machine vmc          a machine preset, or DEFAULT
PREFERENCES SET updates on           ON or OFF
PREFERENCES SET update-url https://example.com/sicdm/latest.json
PREFERENCES RESET                    back to the defaults (the recent files stay)
```

**Where they are kept:**

| Platform | Location |
| --- | --- |
| Linux | `$XDG_CONFIG_HOME/sicdm/preferences.json` (`~/.config/sicdm/preferences.json`) |
| macOS | `~/Library/Application Support/SI-CDM/preferences.json` |
| Windows | `%APPDATA%\SI-CDM\preferences.json` |

Set `SICDM_PREFERENCES` to use another file. A value in the file that SI CDM wouldn't accept
from `PREFERENCES SET` is ignored, and the default is used.

## Autosave and recovery

While a project has unsaved changes, SI CDM keeps a recovery copy of it, written once the
changes are as old as the autosave setting (a minute by default) and again after further
changes. Saving, closing or quitting normally removes the copy. If SI CDM stops
unexpectedly (a crash, a power cut, a killed process), the next start offers the work back:
*Restore All* opens it in new tabs, *Discard* deletes it, *Later* asks again next time.

```text
RECOVER              the copies left by sessions that ended
RECOVER 1            open the first
RECOVER DISCARD ALL  delete them
```

Recovery copies are kept in `%LOCALAPPDATA%\SI-CDM\Recovery` (Windows),
`~/Library/Application Support/SI-CDM/Recovery` (macOS) or `~/.local/state/sicdm/recovery`
(Linux); set `SICDM_RECOVERY_DIR` to use another folder.

## Recent files

*File → Open Recent* lists the last ten projects opened or saved, newest first. A project
that has been moved or deleted is greyed out; hover over a name for its folder.

```text
RECENT            the list
RECENT 2          open the second
RECENT CLEAR      empty the list
```

## Checking for updates

SI CDM has no built-in update server: your organisation publishes a small *release feed*, a
JSON file on a web server or a shared drive, and sets its address as the `update-url`
preference:

```json
{
  "version": "1.3.0",
  "date": "2026-11-02",
  "notes": "Lathe threading cycles; faster STEP import",
  "url": "https://example.com/sicdm/1.3.0/",
  "downloads": {"windows": "https://example.com/sicdm/1.3.0/SI-CDM-1.3.0-win64.exe",
                "macos": "https://example.com/sicdm/1.3.0/SI-CDM-1.3.0.dmg",
                "linux": "https://example.com/sicdm/1.3.0/SI-CDM-1.3.0-Linux.tar.gz"}
}
```

*Help → Check for Updates*, *Check Now* in Preferences, or `UPDATES` reads the feed and says
whether a newer version is out, and where to download it. With *Check for a newer release
when SI CDM starts* on, the check runs in the background at start, and a newer release is
shown in the status bar.

`https://` and `http://` feeds are fetched with the system's `curl` program (part of
Windows 10 and later, macOS and Linux); `file://` addresses (`file:///S:/software/sicdm/latest.json`)
are read directly. Nothing about the computer or its user is sent. SI CDM never downloads or
installs an update by itself.
