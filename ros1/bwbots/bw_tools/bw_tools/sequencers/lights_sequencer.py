import csv
from .sequence_generator import SequenceGenerator, BwSequenceType


class LightsSequencer:
    def __init__(self, file_path) -> None:
        self.raw_sequence = self.load_file(file_path)
        self.show_delay = 35  # millisecond delay introduced by show command
    
    def generate(self, generator: SequenceGenerator):
        for element_type, parameters in self.raw_sequence:
            if element_type == BwSequenceType.START_TONE:
                channel = parameters[0]
                volume = parameters[1]
                frequency = parameters[2]
                generator.add(SequenceGenerator.make_start_tone(channel, frequency, volume))
            elif element_type == BwSequenceType.STOP_TONE:
                channel = parameters[0]
                generator.add(SequenceGenerator.make_stop_tone(channel))
            elif element_type == BwSequenceType.DELAY:
                delay = parameters[0]
                generator.add(SequenceGenerator.make_delay(delay))
            elif element_type == BwSequenceType.SET_RING_LED:
                index = parameters[0]
                r = parameters[1]
                g = parameters[2]
                b = parameters[3]
                w = parameters[4]
                if len(parameters) == 6 and type(parameters[5]) == int:
                    stop_index = parameters[5]
                    start_index = index
                    for index in range(start_index, stop_index + 1):
                        generator.add(SequenceGenerator.make_led(index, r, g, b, w))
                else:
                    generator.add(SequenceGenerator.make_led(index, r, g, b, w))
            elif element_type == BwSequenceType.SHOW_LED:
                if len(parameters) != 0 and type(parameters[0]) == int:
                    delay = parameters[0]
                    delay -= self.show_delay
                    delay = max(0, min(0xffff, int(delay)))
                    if delay > 0:
                        generator.add(SequenceGenerator.make_delay(delay))
                generator.add(SequenceGenerator.make_show())

    def load_file(self, file_path):
        raw_sequence = []
        with open(file_path) as file:
            reader = csv.reader(file)
            header = next(reader)
            for row in reader:
                if len(row) == 0 or len(row[0]) == 0:
                    continue
                element_type = int(row[0])
                values = []
                for element in row[1:]:
                    try:
                        element = int(element)
                    except ValueError:
                        pass
                    values.append(element)
                parameters = tuple(values)
                raw_sequence.append((element_type, parameters))
        return raw_sequence

