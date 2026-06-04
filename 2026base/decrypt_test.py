from Crypto.Cipher import AES
from Crypto.Protocol.KDF import PBKDF2
from Crypto.Util.Padding import unpad
from Crypto.Hash import SHA256
import base64

password: str = "1234"

encoded: str = "U2FsdGVkX1/0poQ1faBD6kE2VwTuJ8b4FplslO2HMAQ="

# remove whitespace/newlines safely
encoded_bytes = b"".join(encoded.encode('utf-8').split())

data = base64.b64decode(encoded)

if data[:8] != b"Salted__":
    raise ValueError("Missing OpenSSL salt header")

salt = data[8:16]
ciphertext = data[16:]
password_bytes = b"".join(password.encode('utf-8').split())

# IMPORTANT:
# OpenSSL -pbkdf2 defaults:
#   iterations = 10000
#   hash = SHA256
key = PBKDF2(
    password_bytes,
    salt,
    dkLen=16,
    count=10000,
    hmac_hash_module=SHA256
)

cipher = AES.new(key, AES.MODE_ECB)

plaintext = unpad(
    cipher.decrypt(ciphertext),
    16
)

print(plaintext.decode("utf-8"))