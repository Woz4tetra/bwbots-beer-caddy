import os
import re
import yaml
import argparse
import textwrap
from bw_tools.sequencers import LightsSequencer, SequenceGenerator, MidiSequencer

def main():
    parser = argparse.ArgumentParser(description="stored_sequence_generator")

    parser.add_argument("filepaths", nargs='+',
                        help="Path to sequence file")
    parser.add_argument("--config",
                        default="config.yaml",
                        help="sequence config file")
    args = parser.parse_args()
    
    stored_sequences_file_path = os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "../../firmware/bw_bcause/include/StoredSequences.h"
    )
    file_columns = 120

    with open(args.config) as file:
        config = yaml.safe_load(file)
        print(config)

    sequence_locations = []
    stored_sequences = []
    paths = []
    names = []
    num_channels = 4
    for filepath in args.filepaths:
        generator = SequenceGenerator()
        name = os.path.splitext(os.path.basename(filepath))[0]
        if filepath.endswith(".csv"):
            lights = LightsSequencer(filepath)
            lights.generate(generator)
        elif filepath.endswith(".mid"):
            if name in config:
                volume = config[name].get("volume", 45)
                tempo_multiplier = config[name].get("tempo_multiplier", 1.0)
                tracks = config[name].get("tracks", [0, 1, 2, 3, 4, 5, 6, 7])
                num_leds = config[name].get("num_leds", None)
                channel_dithering = config[name].get("channel_dithering", 2)
                dither_delay = config[name].get("dither_delay", 0.05)
            else:
                volume = 45
                tempo_multiplier = 1.0
                tracks = [0, 1, 2, 3, 4, 5, 6, 7]
                num_leds = None
                channel_dithering = 2
                dither_delay = 0.05
            print(f"Set volume to {volume}. Set tempo multipler to {tempo_multiplier}")
            midi_sequencer = MidiSequencer(
                filepath,
                int(volume),
                channel_dithering,
                dither_delay,
                float(tempo_multiplier),
                tracks,
                num_leds,
                num_channels
            )
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
#pragma once
#include <Arduino.h>

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

uint64_t get_stored_element(uint8_t serial, uint64_t index);
""" % (
    "\n    ".join(textwrap.wrap(", ".join(["0x%02x" % x for x in stored_sequences]), width=file_columns)),
    "\n    ".join(textwrap.wrap(", ".join(["0x%02x" % x for x in sequence_locations]), width=file_columns)),
    len(sequence_locations),
    "\n".join(["    %s: %s" % (index, path) for index, path in enumerate(paths)]),
    "\n".join(["const uint8_t STORED_SEQUENCE_%s = %s;" % (re.sub(r"[\s\W]", "_", name.upper()), index) for index, name in enumerate(names)])
)
    print(f"Generated code for {len(sequence_locations)} sequences. Total length is {len(stored_sequences)}")
    
    with open(stored_sequences_file_path, 'w') as file:
        file.write(code)

main()
