// SHA-512 (FIPS 180-4) and Ed25519 (RFC 8032).
//
// Field elements mod p = 2^255 - 19 are 16 limbs of 16 bits in 64-bit integers, carried
// after each product; points are extended twisted Edwards coordinates (X, Y, Z, T). The
// curve's constants (d, sqrt(-1) and the base point) are derived from their definitions
// at first use rather than written out.

#if defined(_WIN32)
#define _CRT_RAND_S
#endif

#include "core/Ed25519.hpp"

#include <cstdlib>
#include <cstring>
#include <fstream>

namespace sicdm {

// --- SHA-512 ------------------------------------------------------------------------------

namespace {

constexpr std::uint64_t kK[80] = {
    0x428a2f98d728ae22ULL, 0x7137449123ef65cdULL, 0xb5c0fbcfec4d3b2fULL, 0xe9b5dba58189dbbcULL, 0x3956c25bf348b538ULL, 0x59f111f1b605d019ULL,
    0x923f82a4af194f9bULL, 0xab1c5ed5da6d8118ULL, 0xd807aa98a3030242ULL, 0x12835b0145706fbeULL, 0x243185be4ee4b28cULL, 0x550c7dc3d5ffb4e2ULL,
    0x72be5d74f27b896fULL, 0x80deb1fe3b1696b1ULL, 0x9bdc06a725c71235ULL, 0xc19bf174cf692694ULL, 0xe49b69c19ef14ad2ULL, 0xefbe4786384f25e3ULL,
    0x0fc19dc68b8cd5b5ULL, 0x240ca1cc77ac9c65ULL, 0x2de92c6f592b0275ULL, 0x4a7484aa6ea6e483ULL, 0x5cb0a9dcbd41fbd4ULL, 0x76f988da831153b5ULL,
    0x983e5152ee66dfabULL, 0xa831c66d2db43210ULL, 0xb00327c898fb213fULL, 0xbf597fc7beef0ee4ULL, 0xc6e00bf33da88fc2ULL, 0xd5a79147930aa725ULL,
    0x06ca6351e003826fULL, 0x142929670a0e6e70ULL, 0x27b70a8546d22ffcULL, 0x2e1b21385c26c926ULL, 0x4d2c6dfc5ac42aedULL, 0x53380d139d95b3dfULL,
    0x650a73548baf63deULL, 0x766a0abb3c77b2a8ULL, 0x81c2c92e47edaee6ULL, 0x92722c851482353bULL, 0xa2bfe8a14cf10364ULL, 0xa81a664bbc423001ULL,
    0xc24b8b70d0f89791ULL, 0xc76c51a30654be30ULL, 0xd192e819d6ef5218ULL, 0xd69906245565a910ULL, 0xf40e35855771202aULL, 0x106aa07032bbd1b8ULL,
    0x19a4c116b8d2d0c8ULL, 0x1e376c085141ab53ULL, 0x2748774cdf8eeb99ULL, 0x34b0bcb5e19b48a8ULL, 0x391c0cb3c5c95a63ULL, 0x4ed8aa4ae3418acbULL,
    0x5b9cca4f7763e373ULL, 0x682e6ff3d6b2b8a3ULL, 0x748f82ee5defb2fcULL, 0x78a5636f43172f60ULL, 0x84c87814a1f0ab72ULL, 0x8cc702081a6439ecULL,
    0x90befffa23631e28ULL, 0xa4506cebde82bde9ULL, 0xbef9a3f7b2c67915ULL, 0xc67178f2e372532bULL, 0xca273eceea26619cULL, 0xd186b8c721c0c207ULL,
    0xeada7dd6cde0eb1eULL, 0xf57d4f7fee6ed178ULL, 0x06f067aa72176fbaULL, 0x0a637dc5a2c898a6ULL, 0x113f9804bef90daeULL, 0x1b710b35131c471bULL,
    0x28db77f523047d84ULL, 0x32caab7b40c72493ULL, 0x3c9ebe0a15c9bebcULL, 0x431d67c49c100d4cULL, 0x4cc5d4becb3e42b6ULL, 0x597f299cfc657e2aULL,
    0x5fcb6fab3ad6faecULL, 0x6c44198c4a475817ULL};

std::uint64_t rotr(std::uint64_t x, int n) { return (x >> n) | (x << (64 - n)); }

void compress(std::uint64_t h[8], const std::uint8_t block[128]) {
    std::uint64_t w[80];
    for (int i = 0; i < 16; ++i) {
        w[i] = 0;
        for (int b = 0; b < 8; ++b) w[i] = (w[i] << 8) | block[i * 8 + b];
    }
    for (int i = 16; i < 80; ++i) {
        const std::uint64_t s0 = rotr(w[i - 15], 1) ^ rotr(w[i - 15], 8) ^ (w[i - 15] >> 7);
        const std::uint64_t s1 = rotr(w[i - 2], 19) ^ rotr(w[i - 2], 61) ^ (w[i - 2] >> 6);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint64_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 80; ++i) {
        const std::uint64_t S1 = rotr(e, 14) ^ rotr(e, 18) ^ rotr(e, 41);
        const std::uint64_t ch = (e & f) ^ (~e & g);
        const std::uint64_t t1 = hh + S1 + ch + kK[i] + w[i];
        const std::uint64_t S0 = rotr(a, 28) ^ rotr(a, 34) ^ rotr(a, 39);
        const std::uint64_t maj = (a & b) ^ (a & c) ^ (b & c);
        const std::uint64_t t2 = S0 + maj;
        hh = g, g = f, f = e, e = d + t1, d = c, c = b, b = a, a = t1 + t2;
    }
    h[0] += a, h[1] += b, h[2] += c, h[3] += d, h[4] += e, h[5] += f, h[6] += g, h[7] += hh;
}

} // namespace

std::array<std::uint8_t, 64> sha512(const std::uint8_t* data, std::size_t size) {
    std::uint64_t h[8] = {0x6a09e667f3bcc908ULL, 0xbb67ae8584caa73bULL, 0x3c6ef372fe94f82bULL, 0xa54ff53a5f1d36f1ULL,
                          0x510e527fade682d1ULL, 0x9b05688c2b3e6c1fULL, 0x1f83d9abfb41bd6bULL, 0x5be0cd19137e2179ULL};
    std::size_t i = 0;
    for (; i + 128 <= size; i += 128) compress(h, data + i);
    std::uint8_t tail[256] = {};
    const std::size_t rest = size - i;
    if (rest) std::memcpy(tail, data + i, rest);
    tail[rest] = 0x80;
    const std::size_t blocks = rest + 1 + 16 <= 128 ? 1 : 2;
    const std::uint64_t bits = static_cast<std::uint64_t>(size) * 8; // the upper 64 bits of the length stay 0
    for (int b = 0; b < 8; ++b) tail[blocks * 128 - 1 - b] = static_cast<std::uint8_t>(bits >> (8 * b));
    for (std::size_t k = 0; k < blocks; ++k) compress(h, tail + k * 128);
    std::array<std::uint8_t, 64> out{};
    for (int k = 0; k < 8; ++k)
        for (int b = 0; b < 8; ++b) out[static_cast<std::size_t>(k * 8 + b)] = static_cast<std::uint8_t>(h[k] >> (56 - 8 * b));
    return out;
}

// --- the field mod 2^255 - 19 --------------------------------------------------------------

namespace {

using i64 = std::int64_t;
struct Fe {
    i64 v[16] = {};
};

void carry(Fe& o) {
    for (int i = 0; i < 16; ++i) {
        o.v[i] += i64{1} << 16;
        const i64 c = o.v[i] >> 16;
        if (i < 15) o.v[i + 1] += c - 1;
        else o.v[0] += 38 * (c - 1); // 2^256 = 38 mod p
        o.v[i] -= c * (i64{1} << 16);
    }
}

// Swaps p and q when b is 1, in constant time.
void select(Fe& p, Fe& q, int b) {
    const i64 c = ~(static_cast<i64>(b) - 1);
    for (int i = 0; i < 16; ++i) {
        const i64 t = c & (p.v[i] ^ q.v[i]);
        p.v[i] ^= t;
        q.v[i] ^= t;
    }
}

void pack(std::uint8_t out[32], const Fe& n) {
    Fe t = n, m;
    carry(t), carry(t), carry(t);
    for (int j = 0; j < 2; ++j) {
        m.v[0] = t.v[0] - 0xffed;
        for (int i = 1; i < 15; ++i) {
            m.v[i] = t.v[i] - 0xffff - ((m.v[i - 1] >> 16) & 1);
            m.v[i - 1] &= 0xffff;
        }
        m.v[15] = t.v[15] - 0x7fff - ((m.v[14] >> 16) & 1);
        const int b = static_cast<int>((m.v[15] >> 16) & 1);
        m.v[14] &= 0xffff;
        select(t, m, 1 - b);
    }
    for (int i = 0; i < 16; ++i) {
        out[2 * i] = static_cast<std::uint8_t>(t.v[i] & 0xff);
        out[2 * i + 1] = static_cast<std::uint8_t>((t.v[i] >> 8) & 0xff);
    }
}

Fe unpack(const std::uint8_t in[32]) {
    Fe o;
    for (int i = 0; i < 16; ++i) o.v[i] = in[2 * i] + (i64{in[2 * i + 1]} << 8);
    o.v[15] &= 0x7fff;
    return o;
}

Fe add(const Fe& a, const Fe& b) {
    Fe o;
    for (int i = 0; i < 16; ++i) o.v[i] = a.v[i] + b.v[i];
    return o;
}
Fe sub(const Fe& a, const Fe& b) {
    Fe o;
    for (int i = 0; i < 16; ++i) o.v[i] = a.v[i] - b.v[i];
    return o;
}
Fe mul(const Fe& a, const Fe& b) {
    i64 t[31] = {};
    for (int i = 0; i < 16; ++i)
        for (int j = 0; j < 16; ++j) t[i + j] += a.v[i] * b.v[j];
    for (int i = 0; i < 15; ++i) t[i] += 38 * t[i + 16];
    Fe o;
    for (int i = 0; i < 16; ++i) o.v[i] = t[i];
    carry(o), carry(o);
    return o;
}
Fe sq(const Fe& a) { return mul(a, a); }
Fe small(i64 v) {
    Fe o;
    o.v[0] = v;
    return o;
}

// a^(p-2) = 1/a: p-2 = 2^255 - 21, every bit set but bits 2 and 4.
Fe invert(const Fe& a) {
    Fe c = a;
    for (int i = 253; i >= 0; --i) {
        c = sq(c);
        if (i != 2 && i != 4) c = mul(c, a);
    }
    return c;
}

// a^((p-5)/8) = a^(2^252 - 3): every bit set but bit 1.
Fe pow2523(const Fe& a) {
    Fe c = a;
    for (int i = 250; i >= 0; --i) {
        c = sq(c);
        if (i != 1) c = mul(c, a);
    }
    return c;
}

bool equal(const Fe& a, const Fe& b) {
    std::uint8_t x[32], y[32];
    pack(x, a), pack(y, b);
    return std::memcmp(x, y, 32) == 0;
}
int parity(const Fe& a) {
    std::uint8_t d[32];
    pack(d, a);
    return d[0] & 1;
}

struct Curve {
    Fe d, d2, sqrtm1;
    Fe bx, by;
};

const Curve& curve() {
    static const Curve c = [] {
        Curve k;
        k.d = mul(sub(Fe{}, small(121665)), invert(small(121666))); // -121665 / 121666
        k.d2 = add(k.d, k.d);
        // sqrt(-1) = 2^((p-1)/4); (p-1)/4 = 2^253 - 5: every bit set but bit 2.
        Fe two = small(2), r = two;
        for (int i = 251; i >= 0; --i) {
            r = sq(r);
            if (i != 2) r = mul(r, two);
        }
        k.sqrtm1 = r;
        // The base point: y = 4/5, x the even root.
        k.by = mul(small(4), invert(small(5)));
        const Fe y2 = sq(k.by);
        const Fe x2 = mul(sub(y2, small(1)), invert(add(mul(k.d, y2), small(1))));
        // A root of x2 is x2^((p+3)/8) = x2 * x2^((p-5)/8), or that times sqrt(-1).
        Fe xr = mul(x2, pow2523(x2));
        if (!equal(sq(xr), x2)) xr = mul(xr, k.sqrtm1);
        if (parity(xr) != 0) xr = sub(Fe{}, xr);
        k.bx = xr;
        return k;
    }();
    return c;
}

struct Pt {
    Fe x, y, z, t;
};

Pt ptAdd(const Pt& p, const Pt& q) {
    const Curve& k = curve();
    const Fe a = mul(sub(p.y, p.x), sub(q.y, q.x));
    const Fe b = mul(add(p.x, p.y), add(q.x, q.y));
    const Fe c = mul(mul(p.t, q.t), k.d2);
    Fe d = mul(p.z, q.z);
    d = add(d, d);
    const Fe e = sub(b, a), f = sub(d, c), g = add(d, c), h = add(b, a);
    return {mul(e, f), mul(h, g), mul(g, f), mul(e, h)};
}

void ptSwap(Pt& p, Pt& q, int b) {
    select(p.x, q.x, b), select(p.y, q.y, b), select(p.z, q.z, b), select(p.t, q.t, b);
}

void ptPack(std::uint8_t out[32], const Pt& p) {
    const Fe zi = invert(p.z);
    const Fe x = mul(p.x, zi), y = mul(p.y, zi);
    pack(out, y);
    out[31] ^= static_cast<std::uint8_t>(parity(x) << 7);
}

Pt scalarMul(Pt q, const std::uint8_t s[32]) {
    Pt p{Fe{}, small(1), small(1), Fe{}};
    for (int i = 255; i >= 0; --i) {
        const int b = (s[i / 8] >> (i & 7)) & 1;
        ptSwap(p, q, b);
        q = ptAdd(q, p);
        p = ptAdd(p, p);
        ptSwap(p, q, b);
    }
    return p;
}

Pt scalarBase(const std::uint8_t s[32]) {
    const Curve& k = curve();
    return scalarMul({k.bx, k.by, small(1), mul(k.bx, k.by)}, s);
}

// The point a key encodes, negated (what verification wants). False if it isn't on the curve.
bool unpackNeg(Pt& r, const std::uint8_t p[32]) {
    const Curve& k = curve();
    r.z = small(1);
    r.y = unpack(p);
    Fe num = sq(r.y);
    Fe den = mul(num, k.d);
    num = sub(num, r.z);
    den = add(r.z, den);
    const Fe den2 = sq(den), den4 = sq(den2), den6 = mul(den4, den2);
    Fe t = mul(mul(den6, num), den);
    t = pow2523(t);
    t = mul(mul(mul(t, num), den), den);
    r.x = mul(t, den);
    Fe chk = mul(sq(r.x), den);
    if (!equal(chk, num)) r.x = mul(r.x, k.sqrtm1);
    chk = mul(sq(r.x), den);
    if (!equal(chk, num)) return false;
    if (parity(r.x) == (p[31] >> 7)) r.x = sub(Fe{}, r.x);
    r.t = mul(r.x, r.y);
    return true;
}

// The group's order L, little endian.
constexpr i64 kL[32] = {0xed, 0xd3, 0xf5, 0x5c, 0x1a, 0x63, 0x12, 0x58, 0xd6, 0x9c, 0xf7, 0xa2, 0xde, 0xf9, 0xde, 0x14,
                        0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0,    0x10};

// r = x mod L for a 64-byte x.
void modL(std::uint8_t r[32], i64 x[64]) {
    for (int i = 63; i >= 32; --i) {
        i64 c = 0;
        int j = i - 32;
        for (; j < i - 12; ++j) {
            x[j] += c - 16 * x[i] * kL[j - (i - 32)];
            c = (x[j] + 128) >> 8;
            x[j] -= c * 256;
        }
        x[j] += c;
        x[i] = 0;
    }
    i64 c = 0;
    for (int j = 0; j < 32; ++j) {
        x[j] += c - (x[31] >> 4) * kL[j];
        c = x[j] >> 8;
        x[j] &= 255;
    }
    for (int j = 0; j < 32; ++j) x[j] -= c * kL[j];
    for (int i = 0; i < 32; ++i) {
        x[i + 1] += x[i] >> 8;
        r[i] = static_cast<std::uint8_t>(x[i] & 255);
    }
}

void reduce(std::uint8_t r[64]) {
    i64 x[64];
    for (int i = 0; i < 64; ++i) x[i] = r[i];
    std::memset(r, 0, 64);
    modL(r, x);
}

std::array<std::uint8_t, 64> hashParts(std::initializer_list<std::pair<const std::uint8_t*, std::size_t>> parts) {
    std::vector<std::uint8_t> all;
    for (const auto& [p, n] : parts) all.insert(all.end(), p, p + n);
    return sha512(all.data(), all.size());
}

} // namespace

Ed25519PublicKey ed25519PublicKey(const Ed25519Seed& seed) {
    std::array<std::uint8_t, 64> d = sha512(seed.data(), seed.size());
    d[0] &= 248, d[31] &= 127, d[31] |= 64;
    Ed25519PublicKey pk{};
    ptPack(pk.data(), scalarBase(d.data()));
    return pk;
}

Ed25519Signature ed25519Sign(const Ed25519Seed& seed, const std::uint8_t* m, std::size_t n) {
    std::array<std::uint8_t, 64> d = sha512(seed.data(), seed.size());
    d[0] &= 248, d[31] &= 127, d[31] |= 64;
    const Ed25519PublicKey pk = ed25519PublicKey(seed);
    std::array<std::uint8_t, 64> r = hashParts({{d.data() + 32, 32}, {m, n}});
    reduce(r.data());
    Ed25519Signature sig{};
    ptPack(sig.data(), scalarBase(r.data()));
    std::array<std::uint8_t, 64> h = hashParts({{sig.data(), 32}, {pk.data(), 32}, {m, n}});
    reduce(h.data());
    i64 x[64] = {};
    for (int i = 0; i < 32; ++i) x[i] = r[static_cast<std::size_t>(i)];
    for (int i = 0; i < 32; ++i)
        for (int j = 0; j < 32; ++j) x[i + j] += i64{h[static_cast<std::size_t>(i)]} * i64{d[static_cast<std::size_t>(j)]};
    modL(sig.data() + 32, x);
    return sig;
}

bool ed25519Verify(const Ed25519PublicKey& key, const std::uint8_t* m, std::size_t n, const Ed25519Signature& sig) {
    // S must be below L.
    for (int i = 31; i >= 0; --i) {
        const i64 s = sig[static_cast<std::size_t>(32 + i)];
        if (s < kL[i]) break;
        if (s > kL[i] || i == 0) return false;
    }
    Pt q;
    if (!unpackNeg(q, key.data())) return false;
    std::array<std::uint8_t, 64> h = hashParts({{sig.data(), 32}, {key.data(), 32}, {m, n}});
    reduce(h.data());
    const Pt p = ptAdd(scalarMul(q, h.data()), scalarBase(sig.data() + 32));
    std::uint8_t t[32];
    ptPack(t, p);
    return std::memcmp(t, sig.data(), 32) == 0;
}

bool ed25519RandomSeed(Ed25519Seed& out) {
#if defined(_WIN32)
    for (std::size_t i = 0; i < out.size(); i += 4) {
        unsigned int v = 0;
        if (rand_s(&v) != 0) return false;
        for (std::size_t b = 0; b < 4 && i + b < out.size(); ++b) out[i + b] = static_cast<std::uint8_t>(v >> (8 * b));
    }
    return true;
#else
    std::ifstream f("/dev/urandom", std::ios::binary);
    return f && f.read(reinterpret_cast<char*>(out.data()), static_cast<std::streamsize>(out.size())) && f.gcount() == static_cast<std::streamsize>(out.size());
#endif
}

std::string toHex(const std::uint8_t* data, std::size_t size) {
    static const char kDigits[] = "0123456789abcdef";
    std::string s;
    for (std::size_t i = 0; i < size; ++i) s += kDigits[data[i] >> 4], s += kDigits[data[i] & 15];
    return s;
}

bool fromHex(const std::string& hex, std::uint8_t* out, std::size_t size) {
    if (hex.size() != size * 2) return false;
    auto val = [](char c) {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    for (std::size_t i = 0; i < size; ++i) {
        const int a = val(hex[2 * i]), b = val(hex[2 * i + 1]);
        if (a < 0 || b < 0) return false;
        out[i] = static_cast<std::uint8_t>(a * 16 + b);
    }
    return true;
}

} // namespace sicdm
