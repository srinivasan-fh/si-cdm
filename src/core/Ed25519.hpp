#pragma once

// SHA-512 (FIPS 180-4) and Ed25519 signatures (RFC 8032), for licence files.
//
// Ed25519 here is the plain RFC 8032 scheme: a 32-byte secret seed, a 32-byte public key
// and 64-byte signatures. Verification rejects non-canonical S (S >= L).

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace sicdm {

std::array<std::uint8_t, 64> sha512(const std::uint8_t* data, std::size_t size);
inline std::array<std::uint8_t, 64> sha512(const std::string& s) {
    return sha512(reinterpret_cast<const std::uint8_t*>(s.data()), s.size());
}

using Ed25519Seed = std::array<std::uint8_t, 32>;
using Ed25519PublicKey = std::array<std::uint8_t, 32>;
using Ed25519Signature = std::array<std::uint8_t, 64>;

Ed25519PublicKey ed25519PublicKey(const Ed25519Seed& seed);
Ed25519Signature ed25519Sign(const Ed25519Seed& seed, const std::uint8_t* message, std::size_t size);
bool ed25519Verify(const Ed25519PublicKey& key, const std::uint8_t* message, std::size_t size, const Ed25519Signature& sig);

// A seed from the system's random source. False if none could be read.
bool ed25519RandomSeed(Ed25519Seed& out);

// Hex text and back (lower case; false on a bad character or length).
std::string toHex(const std::uint8_t* data, std::size_t size);
bool fromHex(const std::string& hex, std::uint8_t* out, std::size_t size);

} // namespace sicdm
