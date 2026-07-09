#!/usr/bin/env bash
# setup-signing-cert.sh - Create a stable self-signed code-signing
# certificate in the user's login keychain so that subsequent
# hot-install runs sign opentrack with a CONSTANT code-signature hash
# instead of changing it on every ad-hoc resign.
#
# Why this matters
# ----------------
# macOS TCC (Privacy & Security: Screen Recording, Input Monitoring,
# Camera, etc.) keys grants on the app's "Designated Requirement",
# which for ad-hoc-signed apps is the per-build random hash. Every
# time we run `codesign --force -s -`, that hash changes, TCC treats
# the bundle as a new app, and the user has to re-grant permissions.
#
# A self-signed cert solves this: the cert's identity is stable, so
# subsequent codesign runs with that identity produce a stable
# Designated Requirement, and TCC keeps the grants forever.
#
# Run this once. After that, hot-install.sh will detect the cert and
# use it automatically.

set -euo pipefail

CERT_NAME="opentrack-dev"
KEYCHAIN="$HOME/Library/Keychains/login.keychain-db"

if security find-identity -v -p codesigning "$KEYCHAIN" 2>/dev/null \
    | grep -Fq "\"$CERT_NAME\""; then
    echo "[setup] '$CERT_NAME' code-signing identity already exists in $KEYCHAIN"
    echo "[setup] Nothing to do. Hot-install will use it."
    exit 0
fi

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "[setup] generating keypair + self-signed cert"
cat > "$TMP/req.cnf" <<'EOF'
[req]
default_bits = 2048
prompt = no
default_md = sha256
distinguished_name = dn
x509_extensions = v3_extensions

[dn]
CN = opentrack-dev

[v3_extensions]
basicConstraints = critical,CA:false
keyUsage = critical,digitalSignature
extendedKeyUsage = critical,codeSigning
EOF

openssl req -new -x509 -days 3650 -nodes \
    -keyout "$TMP/cert.key" \
    -out    "$TMP/cert.crt" \
    -config "$TMP/req.cnf" 2>/dev/null

# macOS Sequoia ships LibreSSL by default, but Homebrew openssl@3 uses
# a default cipher (AES-256-CBC) that macOS's `security` import can't
# read. -legacy + an explicit RC2-40 cipher matches the format that
# Apple's Keychain expects. Fall back to no flags if -legacy is not
# supported (e.g., LibreSSL).
if openssl pkcs12 -help 2>&1 | grep -q -- '-legacy'; then
    P12_LEGACY=(-legacy -keypbe PBE-SHA1-3DES -certpbe PBE-SHA1-3DES)
else
    P12_LEGACY=()
fi
openssl pkcs12 -export "${P12_LEGACY[@]}" \
    -inkey  "$TMP/cert.key" \
    -in     "$TMP/cert.crt" \
    -out    "$TMP/cert.p12" \
    -name   "$CERT_NAME" \
    -password pass:opentrack-dev 2>/dev/null

echo "[setup] importing into login keychain (you may be asked to allow)"
# -T /usr/bin/codesign allows ONLY the codesign binary to use this
# key without prompting. Previously we used -A (any application);
# codesign is the only tool that legitimately needs this key on a
# dev machine, so narrowing the grant reduces the attack surface
# (a rogue app can no longer silently borrow our dev identity).
# Users re-running this script after it was previously set up with
# -A should first `security delete-identity -c opentrack-dev` to
# drop the broader grant, then re-run to get the narrow one.
security import "$TMP/cert.p12" \
    -k "$KEYCHAIN" \
    -P opentrack-dev \
    -T /usr/bin/codesign 2>&1 | tail -3

# Mark the certificate as trusted for code signing in the LOGIN
# keychain. No sudo needed for login-keychain trust settings; only
# affects the current user. Without this step, codesign succeeds
# but the resulting signature is "untrusted", which is fine for
# Gatekeeper bypass via `open` of an unquarantined bundle but
# matters for some TCC interactions.
echo "[setup] adding to trusted code-signing settings (login keychain)"
security add-trusted-cert -p codeSign \
    -k "$KEYCHAIN" \
    "$TMP/cert.crt" 2>&1 | tail -3 || true

echo
security find-identity -v -p codesigning "$KEYCHAIN" | grep "$CERT_NAME" || true
echo
echo "[setup] DONE. Now run:"
echo "    dev/hot-install.sh"
echo "and grant TCC permissions once. They will persist across rebuilds."
