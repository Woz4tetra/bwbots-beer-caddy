import os
import re
import yaml
import argparse
import textwrap
try:
    import pyperclip
    CLIPBOARD = True
except ImportError:
    CLIPBOARD = False
from bw_tools.sequencers import LightsSequencer, SequenceGenerator, MidiSequencer

def main():
    parser = argparse.ArgumentParser(description="stored_sequence_generator")

    parser.add_argument("filepaths", nargs='+',
                        help="Path to sequence file")
    parser.add_argument("--config",
                        default="config.yaml",
                        help="sequence config file")
    args = parser.parse_args()

    with open(args.config) as file:
        config = yaml.safe_load(file)
        print(config)

    sequence_locations = []
    stored_sequences = []
    paths = []
    names = []
    for filepath in args.filepaths:
        generator = SequenceGenerator()
        name = os.path.splitext(os.path.basename(filepath))[0]
        if filepath.endswith(".csv"):
            lights = LightsSequencer(filepath)
            lights.generate(generator)
        elif filepath.endswith(".mid"):
            if name in config:
                volume = config[name].get("volume", 30)
                tempo_multiplier = config[name].get("tempo_multiplier", 1.0)
                tracks = config[name].get("tracks", None)
            else:
                volume = 30
                tempo_multiplier = 1.0
                tracks = None
            print(f"Set volume to {volume}. Set tempo multipler to {tempo_multiplier}")
            midi_sequencer = MidiSequencer(filepath, int(volume), True, float(tempo_multiplier), tracks)
            midi_sequencer.generate(generator)
        else:
            raise RuntimeError(f"Invalid file type: {filepath}")

        sequence_locations.append(len(stored_sequences))
        stored_sequences.append(len(generator) + 1)
        paths.append(filepath)
        names.append(name)
        print(f"Appending sequence of length {len(generator)}")

        for element in generator.iter():
            stored_sequences.append(element.parameters)
    
    code = """
const uint64_t STORED_SEQUENCES[] = {
    %s
};

const uint64_t SEQUENCE_LOCATIONS[] = {
    %s
};

const uint8_t NUM_STORED_SEQUENCES = %s;

/* 
Sequences:
%s
*/

%s
""" % (
    "\n    ".join(textwrap.wrap(", ".join(["0x%02x" % x for x in stored_sequences]))),
    "\n    ".join(textwrap.wrap(", ".join(["0x%02x" % x for x in sequence_locations]))),
    len(sequence_locations),
    "\n".join(["    %s: %s" % (index, path) for index, path in enumerate(paths)]),
    "\n".join(["const uint8_t STORED_SEQUENCE_%s = %s;" % (re.sub(r"[\s\W]", "_", name.upper()), index) for index, name in enumerate(names)])
)
    print(f"Generated code for {len(sequence_locations)} sequences. Total length is {len(stored_sequences)}")
    if CLIPBOARD:
        pyperclip.copy(code)
        print("Code is pasted into your clipboard. Paste it into StoredSequences.h")
    else:
        print(code)
        print("Paste the above code into StoredSequences.h")

main()
