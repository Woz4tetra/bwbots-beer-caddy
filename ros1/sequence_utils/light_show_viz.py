import math
import time
import pygame
import argparse
from bw_tools.sequencers import LightsSequencer, SequenceGenerator, BwSequenceType

def draw_led(screen, index, led_state, num_leds, width, height, ring_radius, led_radius):
    angle = 2 * math.pi * index / num_leds - math.pi / 2.0
    if num_leds % 2 == 0:
        angle += math.pi * 1 / num_leds

    led_x = int(ring_radius * math.cos(angle) + width / 2)
    led_y = int(ring_radius * math.sin(angle) + height / 2)
    r, g, b, w = led_state
    r = min(255, max(0, r + w))
    g = min(255, max(0, g + w))
    b = min(255, max(0, b + w))
    pygame.draw.circle(screen, (int(r), int(g), int(b)), (led_x, led_y), led_radius)

def main():
    parser = argparse.ArgumentParser(description="light_show_viz")

    parser.add_argument("filepath",
                        help="Path to sequence file")
    parser.add_argument("--num_leds",
                        default=24,
                        help="Number of LEDs in the ring")
    parser.add_argument("--loop",
                        action="store_true",
                        help="Loop the sequence")
    args = parser.parse_args()

    generator = SequenceGenerator()
    lights = LightsSequencer(args.filepath)
    lights.generate(generator)

    pygame.init()

    screen = pygame.display.set_mode([500, 500])

    loop = args.loop
    num_leds = args.num_leds

    ring_radius = 100
    led_radius = 10
    width, height = screen.get_size()

    led_states = []
    def reset_state():
        nonlocal led_states
        led_states = [[0, 0, 0, 0] for _ in range(num_leds)]
    reset_state()

    running = True
    while running:
        screen.fill((0, 0, 0,))
        for index, state in enumerate(led_states):
            draw_led(screen, index, (0, 0, 0, 255), len(led_states), width, height, ring_radius, led_radius + 2)
        for element in generator.iter():
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            parameters = element.parameters
            element_type = parameters & 0b1111
            if element_type == BwSequenceType.SET_RING_LED:
                color = (parameters >> 4) & 0xffffffff
                index = (parameters >> 36) & 0xffff
                r = (color >> 16) & 0xff
                g = (color >> 8) & 0xff
                b = color & 0xff
                w = (color >> 24) & 0xff
                led_states[index] = (r, g, b, w)
                print(f"{index} -> {led_states[index]}")
            elif element_type == BwSequenceType.SHOW_LED:
                for index, state in enumerate(led_states):
                    draw_led(screen, index, state, len(led_states), width, height, ring_radius, led_radius)
                pygame.display.flip()
                time.sleep(0.035)
            elif element_type == BwSequenceType.DELAY:
                delay = (parameters >> 4) & 0xffff
                delay /= 1000.0
                print(f"delay for {delay}s")
                time.sleep(delay)
        if not loop:
            should_loop = False
            while running:
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False
                    if event.type == pygame.KEYUP:
                        if event.key == pygame.K_l:
                            should_loop = True
                if should_loop:
                    print("Looping")
                    reset_state()
                    break
                pygame.display.flip()

    pygame.quit()

main()
