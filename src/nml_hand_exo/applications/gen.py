# generate.py
import hashlib
import random

MASK = (1 << 128) - 1
DIFF = 0x9E3779B97F4A7C15F39CC0605FCEA8C3

def pi(x):
    return int.from_bytes(hashlib.sha256(x.to_bytes(16, 'big')).digest()[:16], 'big')

def expand_key(k):
    return (k * DIFF) & MASK

def encrypt(block, k1, k2):
    x = block ^ expand_key(k1)
    x = pi(x)
    x = x ^ expand_key(k2)
    return x

random.seed(42)
k1 = random.randint(0, (1 << 26) - 1)
k2 = random.randint(0, (1 << 26) - 1)

# 16-byte flag exactly fits the 128-bit block
flag = b"flag{m1tm_n0w!!}" 
P = int.from_bytes(flag, 'big')
C = encrypt(P, k1, k2)

print("# Paste this into the prompt:")
print("P =", P)
print("C =", C)
