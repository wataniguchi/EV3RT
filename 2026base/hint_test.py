from py_etrobo_util import HintType, Hint

g_key = "1234"
g_hint1 = None
g_hint2 = None

def main():
    # sample input (global string variable)
    global g_password
    # latch output (global string variables)
    global g_hint1, g_hint2
    # Sample input
    qr = [ "25,35", "U2FsdGVkX1/0poQ1faBD6kE2VwTuJ8b4FplslO2HMAQ=" ]

    for s in qr:
        hint_type, hint_text = Hint(s).resolve(password=g_key)
        if hint_type == HintType.HINT1:
            g_hint1 = hint_text
        elif hint_type == HintType.HINT2:
            g_hint2 = hint_text

    print(f"Hint1 = {g_hint1}, Hint2 = {g_hint2}")


if __name__ == "__main__":
    main()