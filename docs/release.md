# Releases and signing

How a release is built, signed and published. This is for whoever makes releases. Users only
see the result: installers that their system trusts.

## Building

`scripts/release.sh` builds, tests and packages a platform. Packages land in `dist/<version>/`,
with a `SHA256SUMS-<platform>.txt` beside them.

```text
scripts/release.sh version 1.2.0    set the version (commit CMakeLists.txt)
scripts/release.sh tag              tag v1.2.0 and push it: CI builds and publishes the release
```

A pushed `v*` tag makes CI build macOS, Linux and Windows. It then publishes their packages
(with checksums, and signatures where there are keys) as a GitHub release.

## Signing

Each platform is signed when its keys are present, and packaged unsigned otherwise. Every
signature is checked straight after it is made: a bad one stops the release.

| Platform | What is signed | How |
| --- | --- | --- |
| macOS | The app and `sicdm-mcp` (hardened runtime), then the `.dmg` | `codesign` with a Developer ID; then notarized by Apple, stapled, and checked with `spctl` |
| Windows | `sicdm.exe`, `sicdm-mcp.exe` and the installer | Authenticode with `osslsigncode`, time-stamped |
| Linux | The `.deb`, the `.tar.gz` and the checksum file | GPG: a detached, armoured `.asc` beside each |

### Keys on your own machine

| Variable | Meaning |
| --- | --- |
| `SICDM_CODESIGN_ID` | macOS: the signing identity, `Developer ID Application: Name (TEAMID)` |
| `SICDM_NOTARY_PROFILE` | macOS: a `notarytool store-credentials` keychain profile |
| `SICDM_NOTARY_KEY`, `SICDM_NOTARY_KEY_ID`, `SICDM_NOTARY_ISSUER` | macOS: an App Store Connect API key (`.p8`), its ID and issuer, in place of the profile |
| `SICDM_WIN_CERT`, `SICDM_WIN_CERT_PASS` | Windows: the code-signing certificate (`.pfx`) and its password |
| `SICDM_TIMESTAMP_URL` | Windows: an RFC 3161 time-stamp server (default DigiCert's) |
| `SICDM_GPG_KEY`, `SICDM_GPG_PASSPHRASE` | Linux: the GPG key's ID or e-mail, and its passphrase if it has one |
| `SICDM_REQUIRE_SIGNING=1` | Refuse to package a platform unsigned |

### Keys in CI

Store them as repository secrets (*Settings → Secrets and variables → Actions*). Each job
picks up its own keys:

| Secret | Contents |
| --- | --- |
| `SICDM_MAC_CERT_P12` | The Developer ID certificate and key, `.p12`, base64 |
| `SICDM_MAC_CERT_PASS` | Its password |
| `SICDM_CODESIGN_ID` | The identity's name |
| `SICDM_NOTARY_KEY_P8` | The App Store Connect API key, `.p8`, base64 |
| `SICDM_NOTARY_KEY_ID`, `SICDM_NOTARY_ISSUER` | Its key ID and issuer ID |
| `SICDM_WIN_CERT_PFX` | The Authenticode certificate and key, `.pfx`, base64 |
| `SICDM_WIN_CERT_PASS` | Its password |
| `SICDM_GPG_KEY_ASC` | The GPG secret key, armoured (`gpg --export-secret-keys --armor`) |
| `SICDM_GPG_KEY_ID`, `SICDM_GPG_PASSPHRASE` | Its ID or e-mail, and its passphrase |

- **The keys stay secret.** They are written only to the runner's temporary folder (and a
  throwaway keychain on macOS), and never into the packages or logs.
- **Requiring signatures:** set the repository *variable* `SICDM_REQUIRE_SIGNING` to `1`.
  Tag builds then fail rather than publish an unsigned package. Branch builds still package
  unsigned, so CI keeps working without the keys.

## Checking a download

```text
sha256sum -c SHA256SUMS-linux.txt                          the files are whole
gpg --verify sicdm_1.2.0_amd64.deb.asc sicdm_1.2.0_amd64.deb   Linux: signed by the release key
spctl --assess --type open --context context:primary-signature SICDM-1.2.0-macos.dmg
osslsigncode verify -in SICDM-1.2.0-windows-x64.exe        (or the file's Properties > Digital Signatures)
```

Publish the GPG public key (`gpg --export --armor`) where users can find it, such as the
project's website or README, so they can import it once.
