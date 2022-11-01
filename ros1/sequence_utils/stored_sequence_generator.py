import argparse
import textwrap
try:
    import pyperclip
    CLIPBOARD = True
except ImportError:
    CLIPBOARD = False
from bw_tools.sequencers import LightsSequencer, SequenceGenerator

def main():
    parser = argparse.ArgumentParser(description="stored_sequence_generator")

    parser.add_argument("filepaths", nargs='+',
                        help="Path to sequence file")
    args = parser.parse_args()

    sequence_locations = []
    stored_sequences = []
    for filepath in args.filepaths:
        generator = SequenceGenerator()
        lights = LightsSequencer(filepath)
        lights.generate(generator)

        sequence_locations.append(len(stored_sequences))
        stored_sequences.append(len(generator))

        for element in generator.iter():
            stored_sequences.append(element.parameters)
    
    code = """
const PROGMEM uint64_t STORED_SEQUENCES[] = {
    %s
};

const PROGMEM uint64_t SEQUENCE_LOCATIONS[] = {
    %s
};

const uint8_t NUM_STORED_SEQUENCES = %s;

""" % (
    "\n    ".join(textwrap.wrap(", ".join(["0x%02x" % x for x in stored_sequences]))),
    "\n    ".join(textwrap.wrap(", ".join(["0x%02x" % x for x in sequence_locations]))),
    len(sequence_locations)
)
    if CLIPBOARD:
        pyperclip.copy(code)
        print("Code is pasted into your clipboard. Paste it into StoredSequences.h")
    else:
        print(code)
        print("Paste the above code into StoredSequences.h")

main()
