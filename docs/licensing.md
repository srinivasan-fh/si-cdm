# Licences, crash reports and notices

## Licences

A licence says who a copy of SI CDM is licensed to: their name and e-mail, the edition, the
number of seats, and when it ends, if it does. The vendor signs these fields with an Ed25519
key, and SI CDM checks the signature offline against the public key it was built with. Nothing
is looked up over the network.

- **Without a licence**, or in a build without a key, SI CDM runs as the **Community edition**.
  Nothing is locked: the edition is shown in *Help → About* and by `LICENCE`. Code that wants
  to treat editions differently can ask for `currentLicence()`.
- **An expired licence** shows as expired, and the copy runs as the Community edition.
- **A changed licence file**, or one signed with another vendor's key, doesn't check out. It is
  refused when installed, and ignored if it is changed afterwards.

*Help → Licence* shows the licence and installs one: *Install a Licence File...* checks the file
you pick and puts it in place; *Remove Licence* goes back to the Community edition. The same
from the command bar:

```text
LICENCE                     who this copy is licensed to
LICENCE INSTALL file.json   checks a licence, then puts it in place
LICENCE REMOVE              back to the Community edition
```

**Where the installed licence is kept:**

| Platform | Location |
| --- | --- |
| Linux | `$XDG_CONFIG_HOME/sicdm/licence.json` (`~/.config/sicdm/licence.json`) |
| macOS | `~/Library/Application Support/SI-CDM/licence.json` |
| Windows | `%APPDATA%\SI-CDM\licence.json` |

Set `SICDM_LICENCE` to use another file.

### For the vendor: keys and issuing

The `sicdm-licence` tool is built with SI CDM but isn't installed with it. Use it to make a
key pair and to issue licences:

```text
sicdm-licence keygen vendor.key
    Secret key written to vendor.key: keep it safe, and out of the source tree.
    Public key: 3d4017c3...660c
    Build with: cmake -DSICDM_LICENCE_PUBLIC_KEY=3d4017c3...660c ...

sicdm-licence issue vendor.key --licensee "Ada Lovelace" --email ada@example.com \
    --edition Professional --seats 3 --expires 2027-12-31 --out ada.json

sicdm-licence verify ada.json --public 3d4017c3...660c
    Licensed to Ada Lovelace <ada@example.com> (Professional, 3 seats, until 2027-12-31)
```

- **Builds:** a build checks licences against `SICDM_LICENCE_PUBLIC_KEY`, given to CMake as
  64 hex digits. Without it, the build checks none.
- **The secret key** (`vendor.key`) is the only thing that can issue licences. Keep it off the
  build machines and out of the repository. `keygen` never overwrites an existing key.

**What is signed:** these lines, in this order.

```text
SI CDM licence 1
licensee=<name>
email=<email>
edition=<edition>
seats=<n>
issued=<YYYY-MM-DD>
expires=<YYYY-MM-DD, or empty for no end>
```

**The licence file** is JSON with the same fields and the signature (128 hex digits). A
changed field, or one added or taken away, breaks the signature.

**The signatures** are plain RFC 8032 Ed25519. They are checked against the RFC's test
vectors, and signatures with S at or past the group's order are rejected.

## Crash reports

If SI CDM dies on a fault, it writes a report into the crash folder. Faults include a bad
memory access, an abort, an uncaught exception or an arithmetic trap.

The report holds:

- the fault, and an uncaught exception's message;
- the version, the platform and the time;
- the last command run (typed, from a menu, or from an AI agent over MCP);
- the call stack, as module and offset.
  - **Linux and macOS:** `addr2line -e sicdm 0x264362` turns an offset into a source line in a
    build with symbols.
  - **Windows:** the stack is captured with `CaptureStackBackTrace`.

The report is written with the operating system's own calls into buffers prepared at
startup, so it still works when the program's memory is damaged.

**Nothing is sent anywhere.** The next start shows the new reports in *SI CDM Closed
Unexpectedly*, with a button that copies the report for an issue or an e-mail. Reports that
have been shown are kept as `*.seen.txt`, up to the newest 20.

```text
CRASHES           the reports, newest first
CRASHES SHOW n    prints one
CRASHES CLEAR     deletes them
CRASHES FOLDER    where they are kept
```

**The crash folder:**

| Platform | Location |
| --- | --- |
| Linux | `$XDG_STATE_HOME/sicdm/crashes` (`~/.local/state/sicdm/crashes`) |
| macOS | `~/Library/Application Support/SI-CDM/Crashes` |
| Windows | `%LOCALAPPDATA%\SI-CDM\Crashes` |

Set `SICDM_CRASH_DIR` to use another. The headless MCP server, `sicdm-mcp`, writes reports
too.

Unsaved work at the time of a crash is offered back by recovery (see `RECOVER`).

## Third-party notices

*Help → About → Third-party components*, and the `NOTICES` command, list the components SI
CDM is built with and their licences. The text is `resources/THIRD-PARTY-NOTICES.txt`,
compiled into the program. Packages install it, with the full licence texts, beside the
program.
