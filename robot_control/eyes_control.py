import sys
import pygame
from pygame.surface import Surface
import pygame.gfxdraw
import time


def hex_to_rgb(hex_color):
    return tuple(int(hex_color[i : i + 2], 16) for i in (0, 2, 4))


def draw_rounded_polygon(screen, color, points, radius):
    if len(points) < 3:
        raise ValueError("Polygon must have at least 3 points")

    # Draw the edges
    for i in range(len(points)):
        start = points[i]
        end = points[(i + 1) % len(points)]
        pygame.draw.line(screen, color, start, end, width=radius * 2)

    # Draw the corners
    for point in points:
        pygame.gfxdraw.aacircle(screen, int(point[0]), int(point[1]), radius - 1, color)
        pygame.gfxdraw.filled_circle(
            screen, int(point[0]), int(point[1]), radius - 1, color
        )
    pygame.draw.polygon(screen, color, points)


class Eyes:

    def __init__(self, screen: Surface):
        pygame.init()
        pygame.display.set_caption("Voyager Eyes")
        self.screen = screen
        self.background_color = (0, 0, 0)
        self.eye_color = hex_to_rgb("67faf9")
        self.neutral_width = 150
        self.neutral_height = 130
        self.small_height = 90

        self.sleep_height = 20
        self.squint_height = 30

        self.left_x = self.screen.get_width() // 3 - self.neutral_width // 2
        self.right_x = 2 * self.screen.get_width() // 3 - self.neutral_width // 2
        self.y = self.screen.get_height() // 2 - self.neutral_width // 2

        self.left_eye_position = (self.left_x, self.y)
        self.right_eye_position = (self.right_x, self.y)

        self.sleep_y = self.y + (self.neutral_height - self.sleep_height)
        self.squint_y = self.y + ((self.neutral_height // 2) - self.squint_height)

    def clear(self):
        self.screen.fill(self.background_color)

    def draw_eyes_squint(self):
        left = (self.left_x, self.squint_y, self.neutral_width, self.sleep_height)
        right = (self.right_x, self.squint_y, self.neutral_width, self.sleep_height)

        pygame.draw.rect(
            self.screen,
            self.eye_color,
            left,
            border_radius=15,
        )

        pygame.draw.rect(
            self.screen,
            self.eye_color,
            right,
            border_radius=15,
        )

    def draw_eyes_neutral(self):
        pygame.draw.rect(
            self.screen,
            self.eye_color,
            (*self.left_eye_position, self.neutral_width, self.neutral_height),
            border_radius=15,
        )

        pygame.draw.rect(
            self.screen,
            self.eye_color,
            (*self.right_eye_position, self.neutral_width, self.neutral_height),
            border_radius=15,
        )

    def draw_eyes_sleep(self):
        left = (self.left_x, self.sleep_y, self.neutral_width, self.sleep_height)
        right = (self.right_x, self.sleep_y, self.neutral_width, self.sleep_height)

        pygame.draw.rect(
            self.screen,
            self.eye_color,
            left,
            border_bottom_left_radius=15,
            border_bottom_right_radius=15,
        )

        pygame.draw.rect(
            self.screen,
            self.eye_color,
            right,
            border_bottom_left_radius=15,
            border_bottom_right_radius=15,
        )

    def draw_left(self):
        left = (self.left_x, self.y, self.neutral_width, self.neutral_height)
        right = (
            (
                self.right_x,
                (self.y - ((self.small_height // 2) - (self.neutral_height // 2))),
                self.neutral_width,
                self.small_height,
            ),
        )
        pygame.draw.rect(
            self.screen,
            self.eye_color,
            left,
            border_radius=15,
        )

        pygame.draw.rect(
            self.screen,
            self.eye_color,
            right,
            border_radius=15,
        )

    def draw_right(self):
        left = (
            (
                self.left_x,
                (self.y - ((self.small_height // 2) - (self.neutral_height // 2))),
                self.neutral_width,
                self.small_height,
            ),
        )
        right = (self.right_x, self.y, self.neutral_width, self.neutral_height)
        pygame.draw.rect(
            self.screen,
            self.eye_color,
            left,
            border_radius=15,
        )

        pygame.draw.rect(
            self.screen,
            self.eye_color,
            right,
            border_radius=15,
        )

    def draw_angry(self, very_angry: bool = False):
        slant_height = self.neutral_height - 100
        if very_angry:
            slant_height = self.neutral_height - 70
        left = [
            (self.left_x, self.y),
            (self.left_x + self.neutral_width, self.y + slant_height),
            (self.left_x + self.neutral_width, self.y + self.neutral_height),
            (self.left_x, self.y + self.neutral_height),
        ]
        right = [
            (self.right_x, self.y + slant_height),
            (self.right_x + self.neutral_width, self.y),
            (self.right_x + self.neutral_width, self.y + self.neutral_height),
            (self.right_x, self.y + self.neutral_height),
        ]
        draw_rounded_polygon(self.screen, self.eye_color, left, 15)
        draw_rounded_polygon(self.screen, self.eye_color, right, 15)

    def draw_furious(self):
        self.draw_angry(very_angry=True)


def main():
    width, height = 800, 600
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    eyes = Eyes(screen)

    emotions = {
        "neutral": eyes.draw_eyes_neutral,
        "squinting": eyes.draw_eyes_squint,
        "sleeping": eyes.draw_eyes_sleep,
        "look_left": eyes.draw_left,
        "look_right": eyes.draw_right,
        "angry": eyes.draw_angry,
        "furious": eyes.draw_furious,
    }

    emotion = emotions["neutral"]

    start_time = time.time()

    # Main loop
    running = True
    while running:
        clock.tick(60)
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN
                and (event.key == pygame.K_ESCAPE or event.key == pygame.K_q)
            ):
                running = False

        if time.time() - start_time > 1:
            emotion = emotions["angry"]
        if time.time() - start_time > 3:
            emotion = emotions["furious"]
        # if time.time() - start_time > 4:
        #     emotion = emotions["look_right"]
        # if time.time() - start_time > 7:
        #     emotion = emotions["sleeping"]
        if time.time() - start_time > 10:
            emotion = emotions["neutral"]
            start_time = time.time()

        eyes.clear()

        emotion()

        # Update the display
        pygame.display.flip()

    # Quit pygame
    pygame.quit()
    sys.exit()


if __name__ == "__main__":
    main()
