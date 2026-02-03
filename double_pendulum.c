#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <math.h>
#include <stdio.h>

#define WINDOW_WIDTH 1710
#define WINDOW_HEIGHT 1000
#define FPS 60
#define DT (1.0 / FPS)
#define GRAVITY 981.0
#define RESET_W 100
#define RESET_H 40
#define TRAIL_LENGTH 2000
#define SLIDER_X 150
#define SLIDER_Y (WINDOW_HEIGHT - 50)
#define SLIDER_W 300
#define SLIDER_H 6
#define LEN1_SLIDER_X 500
#define LEN1_SLIDER_Y (WINDOW_HEIGHT - 50)
#define LEN_SLIDER_W 300
#define LEN_SLIDER_H 6

#define LEN2_SLIDER_X 900
#define LEN2_SLIDER_Y (WINDOW_HEIGHT - 50)

#define MIN_LEN 50.0
#define MAX_LEN 300.0

typedef enum {
  SLIDER_NONE,
  SLIDER_TIME,
  SLIDER_LEN1,
  SLIDER_LEN2
} ActiveSlider;

typedef struct {
  SDL_Point points[TRAIL_LENGTH];
  int head;
  int count;
} Trail;

typedef struct {
  double angle1;
  double angle2;
  double omega1;
  double omega2;
  double length1;
  double length2;
  double mass1;
  double mass2;
} DoublePendulum;

typedef struct {
  SDL_Window *window;
  SDL_Renderer *renderer;
  TTF_Font *font;
  int running;
  double time_scale;
  Trail trail1;
  Trail trail2;
} AppState;

typedef struct {
  double dtheta1, domega1;
  double dtheta2, domega2;
} Deriv;

typedef struct {
  double vx, vy;
} Vec2;

static ActiveSlider active_slider = SLIDER_NONE;

double slider_to_value(double t, double min, double max) {
  return min + t * (max - min);
}

double value_to_slider(double v, double min, double max) {
  return (v - min) / (max - min);
}

Vec2 velocity_mass1(const DoublePendulum *p) {
  Vec2 v;
  v.vx = p->length1 * cos(p->angle1) * p->omega1;
  v.vy = -p->length1 * sin(p->angle1) * p->omega1;
  return v;
}

Vec2 velocity_mass2(const DoublePendulum *p) {
  Vec2 v1 = velocity_mass1(p);
  Vec2 v;
  v.vx = v1.vx + p->length2 * cos(p->angle2) * p->omega2;
  v.vy = v1.vy - p->length2 * sin(p->angle2) * p->omega2;
  return v;
}

void draw_vector(SDL_Renderer *r, int x, int y, double vx, double vy,
                 double scale) {
  int x2 = x + (int)(vx * scale);
  int y2 = y + (int)(vy * scale);

  SDL_RenderDrawLine(r, x, y, x2, y2);

  double angle = atan2(y - y2, x2 - x);
  int ah = 14;
  SDL_RenderDrawLine(r, x2, y2, x2 - ah * cos(angle + 0.5),
                     y2 + ah * sin(angle + 0.5));
  SDL_RenderDrawLine(r, x2, y2, x2 - ah * cos(angle - 0.5),
                     y2 + ah * sin(angle - 0.5));
}

void oops() { printf("oops: %s\n", SDL_GetError()); }

void init_trail(Trail *t) {
  t->head = 0;
  t->count = 0;
}

AppState *init_app() {
  AppState *state = malloc(sizeof(AppState));

  if (SDL_Init(SDL_INIT_VIDEO) < 0) {
    oops();
    return NULL;
  }

  if (TTF_Init() == -1) {
    printf("TTF error: %s\n", TTF_GetError());
    return NULL;
  }

  state->font = TTF_OpenFont("DejaVuSans.ttf", 14);
  if (!state->font) {
    printf("Font error: %s\n", TTF_GetError());
  }

  state->time_scale = 1.0;
  init_trail(&state->trail1);
  init_trail(&state->trail2);

  state->window = SDL_CreateWindow(
      "wahoo wee I'm crazy weehoo", SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, SDL_WINDOW_SHOWN);

  if (!state->window) {
    oops();
    SDL_Quit();
    return NULL;
  }

  state->renderer = SDL_CreateRenderer(
      state->window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

  if (!state->renderer) {
    oops();
    SDL_DestroyWindow(state->window);
    SDL_Quit();
    return NULL;
  }

  SDL_SetRenderDrawBlendMode(state->renderer, SDL_BLENDMODE_BLEND);

  state->running = 1;
  return state;
}

void trail_push(Trail *t, int x, int y) {
  t->points[t->head] = (SDL_Point){x, y};
  t->head = (t->head + 1) % TRAIL_LENGTH;
  if (t->count < TRAIL_LENGTH)
    t->count++;
}

void cleanup_app(AppState *state) {
  if (state->renderer) {
    SDL_DestroyRenderer(state->renderer);
  }
  if (state->window) {
    SDL_DestroyWindow(state->window);
  }
  if (state->font) {
    TTF_CloseFont(state->font);
    TTF_Quit();
  }

  SDL_Quit();
  free(state);
}

void draw_text(SDL_Renderer *r, TTF_Font *font, int x, int y,
               const char *text) {
  SDL_Color color = {255, 255, 255, 255};

  SDL_Surface *surf = TTF_RenderText_Blended(font, text, color);
  SDL_Texture *tex = SDL_CreateTextureFromSurface(r, surf);

  SDL_Rect dst = {x, y, surf->w, surf->h};
  SDL_RenderCopy(r, tex, NULL, &dst);

  SDL_FreeSurface(surf);
  SDL_DestroyTexture(tex);
}

DoublePendulum *init_pendulum() {
  DoublePendulum *p = malloc(sizeof(DoublePendulum));
  p->angle1 = M_PI;
  p->angle2 = M_PI / 2;
  p->omega1 = 0.0;
  p->omega2 = 0.0;
  p->length1 = 150.0;
  p->length2 = 150.0;
  p->mass1 = 1.0;
  p->mass2 = 1.0;
  return p;
}

void calculate_accelerations(DoublePendulum *p, double *alpha1,
                             double *alpha2) {
  double m1 = p->mass1;
  double m2 = p->mass2;
  double l1 = p->length1;
  double l2 = p->length2;
  double g = GRAVITY;

  double theta1 = p->angle1;
  double theta2 = p->angle2;
  double omega1 = p->omega1;
  double omega2 = p->omega2;

  double delta = theta2 - theta1;
  double sin_delta = sin(delta);
  double cos_delta = cos(delta);

  double denom1 = (m1 + m2) * l1 - m2 * l1 * cos_delta * cos_delta;
  double denom2 = (l2 / l1) * denom1;

  *alpha1 =
      (m2 * l1 * omega1 * omega1 * sin_delta * cos_delta +
       m2 * g * sin(theta2) * cos_delta +
       m2 * l2 * omega2 * omega2 * sin_delta - (m1 + m2) * g * sin(theta1)) /
      denom1;

  *alpha2 = (-m2 * l2 * omega2 * omega2 * sin_delta * cos_delta +
             (m1 + m2) * g * sin(theta1) * cos_delta -
             (m1 + m2) * l1 * omega1 * omega1 * sin_delta -
             (m1 + m2) * g * sin(theta2)) /
            denom2;
}

Deriv derivatives(const DoublePendulum *p) {
  Deriv d;
  d.dtheta1 = p->omega1;
  d.dtheta2 = p->omega2;
  calculate_accelerations((DoublePendulum *)p, &d.domega1, &d.domega2);
  return d;
}

void update_pendulum(DoublePendulum *p, double scale) {
  double h = DT * scale;

  DoublePendulum s = *p;

  Deriv k1 = derivatives(&s);

  DoublePendulum s2 = s;
  s2.angle1 += 0.5 * h * k1.dtheta1;
  s2.omega1 += 0.5 * h * k1.domega1;
  s2.angle2 += 0.5 * h * k1.dtheta2;
  s2.omega2 += 0.5 * h * k1.domega2;
  Deriv k2 = derivatives(&s2);

  DoublePendulum s3 = s;
  s3.angle1 += 0.5 * h * k2.dtheta1;
  s3.omega1 += 0.5 * h * k2.domega1;
  s3.angle2 += 0.5 * h * k2.dtheta2;
  s3.omega2 += 0.5 * h * k2.domega2;
  Deriv k3 = derivatives(&s3);

  DoublePendulum s4 = s;
  s4.angle1 += h * k3.dtheta1;
  s4.omega1 += h * k3.domega1;
  s4.angle2 += h * k3.dtheta2;
  s4.omega2 += h * k3.domega2;
  Deriv k4 = derivatives(&s4);

  p->angle1 +=
      h / 6.0 * (k1.dtheta1 + 2 * k2.dtheta1 + 2 * k3.dtheta1 + k4.dtheta1);
  p->omega1 +=
      h / 6.0 * (k1.domega1 + 2 * k2.domega1 + 2 * k3.domega1 + k4.domega1);
  p->angle2 +=
      h / 6.0 * (k1.dtheta2 + 2 * k2.dtheta2 + 2 * k3.dtheta2 + k4.dtheta2);
  p->omega2 +=
      h / 6.0 * (k1.domega2 + 2 * k2.domega2 + 2 * k3.domega2 + k4.domega2);
}

void reset_pendulum(DoublePendulum *p) {
  p->angle1 = M_PI;
  p->angle2 = M_PI / 2;
  p->omega1 = 0.0;
  p->omega2 = 0.0;
}

void draw_trail(SDL_Renderer *r, Trail *t, SDL_Color base) {
  if (t->count < 2)
    return;

  for (int i = 0; i < t->count - 1; i++) {
    int idx1 = (t->head - t->count + i + TRAIL_LENGTH) % TRAIL_LENGTH;
    int idx2 = (idx1 + 1) % TRAIL_LENGTH;

    float alpha = (float)i / (float)t->count;
    SDL_SetRenderDrawColor(r, base.r, base.g, base.b, (Uint8)(alpha * 180));

    SDL_RenderDrawLine(r, t->points[idx1].x, t->points[idx1].y,
                       t->points[idx2].x, t->points[idx2].y);
  }
}

void draw_pendulum(AppState *app, SDL_Renderer *renderer, DoublePendulum *p) {
  int center_x = WINDOW_WIDTH / 2;
  int center_y = WINDOW_HEIGHT / 2;

  int x1 = center_x + (int)(p->length1 * sin(p->angle1));
  int y1 = center_y + (int)(p->length1 * cos(p->angle1));

  int x2 = x1 + (int)(p->length2 * sin(p->angle2));
  int y2 = y1 + (int)(p->length2 * cos(p->angle2));
  trail_push(&app->trail1, x1, y1);
  trail_push(&app->trail2, x2, y2);

  Vec2 v1 = velocity_mass1(p);
  Vec2 v2 = velocity_mass2(p);

  double mag1 = hypot(v1.vx, v1.vy);
  double mag2 = hypot(v2.vx, v2.vy);

  SDL_SetRenderDrawColor(renderer, 20, 20, 40, 255);
  SDL_RenderClear(renderer);

  draw_trail(renderer, &app->trail1, (SDL_Color){80, 160, 255, 255});
  draw_trail(renderer, &app->trail2, (SDL_Color){255, 100, 200, 255});

  char buf[64];

  snprintf(buf, sizeof(buf), "|v1| = %.2f", mag1);
  draw_text(renderer, app->font, x1 + 10, y1 - 10, buf);

  snprintf(buf, sizeof(buf), "|v2| = %.2f", mag2);
  draw_text(renderer, app->font, x2 + 10, y2 - 10, buf);

  SDL_SetRenderDrawColor(renderer, 255, 80, 80, 255);
  draw_vector(renderer, x1, y1, v1.vx, v1.vy, 0.15);

  SDL_SetRenderDrawColor(renderer, 255, 120, 120, 255);
  draw_vector(renderer, x2, y2, v2.vx, v2.vy, 0.15);

  SDL_SetRenderDrawColor(renderer, 100, 150, 255, 255);
  SDL_RenderDrawLine(renderer, center_x, center_y, x1, y1);

  SDL_SetRenderDrawColor(renderer, 150, 200, 255, 255);
  SDL_RenderDrawLine(renderer, x1, y1, x2, y2);

  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_Rect pivot_rect = {center_x - 5, center_y - 5, 10, 10};
  SDL_RenderFillRect(renderer, &pivot_rect);

  SDL_SetRenderDrawColor(renderer, 50, 200, 100, 255);
  SDL_Rect mass1_rect = {x1 - 8, y1 - 8, 16, 16};
  SDL_RenderFillRect(renderer, &mass1_rect);

  SDL_SetRenderDrawColor(renderer, 100, 255, 150, 255);
  SDL_Rect mass2_rect = {x2 - 8, y2 - 8, 16, 16};
  SDL_RenderFillRect(renderer, &mass2_rect);

  SDL_Rect reset_btn = {20, WINDOW_HEIGHT - RESET_H - 20, RESET_W, RESET_H};

  SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
  SDL_RenderFillRect(renderer, &reset_btn);

  SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
  SDL_RenderDrawRect(renderer, &reset_btn);
  SDL_Rect track = {SLIDER_X, SLIDER_Y, SLIDER_W, SLIDER_H};
  SDL_SetRenderDrawColor(renderer, 120, 120, 120, 255);
  SDL_RenderFillRect(renderer, &track);

  int knob_x = SLIDER_X + (int)(app->time_scale / 4.0 * SLIDER_W);
  SDL_Rect knob = {knob_x - 6, SLIDER_Y - 6, 12, 18};

  SDL_Rect track1 = {LEN1_SLIDER_X, LEN1_SLIDER_Y, LEN_SLIDER_W, LEN_SLIDER_H};
  SDL_SetRenderDrawColor(renderer, 120, 120, 120, 255);
  SDL_RenderFillRect(renderer, &track1);

  double t1 = value_to_slider(p->length1, MIN_LEN, MAX_LEN);
  int knob1_x = LEN1_SLIDER_X + (int)(t1 * LEN_SLIDER_W);
  SDL_Rect knob1 = {knob1_x - 6, LEN1_SLIDER_Y - 6, 12, 18};
  SDL_SetRenderDrawColor(renderer, 220, 220, 220, 255);
  SDL_RenderFillRect(renderer, &knob1);

  char buf2[64];
  snprintf(buf2, sizeof(buf2), "Length 1: %.0f", p->length1);
  draw_text(renderer, app->font, LEN1_SLIDER_X, LEN1_SLIDER_Y - 20, buf2);

  SDL_Rect track2 = {LEN2_SLIDER_X, LEN2_SLIDER_Y, LEN_SLIDER_W, LEN_SLIDER_H};
  SDL_SetRenderDrawColor(renderer, 120, 120, 120, 255);
  SDL_RenderFillRect(renderer, &track2);

  double t2 = value_to_slider(p->length2, MIN_LEN, MAX_LEN);
  int knob2_x = LEN2_SLIDER_X + (int)(t2 * LEN_SLIDER_W);
  SDL_Rect knob2 = {knob2_x - 6, LEN2_SLIDER_Y - 6, 12, 18};
  SDL_SetRenderDrawColor(renderer, 220, 220, 220, 255);
  SDL_RenderFillRect(renderer, &knob2);

  snprintf(buf2, sizeof(buf2), "Length 2: %.0f", p->length2);
  draw_text(renderer, app->font, LEN2_SLIDER_X, LEN2_SLIDER_Y - 20, buf2);

  SDL_SetRenderDrawColor(renderer, 220, 220, 220, 255);
  SDL_RenderFillRect(renderer, &knob);

  char buf1[64];
  snprintf(buf1, sizeof(buf1), "Speed: %.2fx", app->time_scale);
  draw_text(renderer, app->font, SLIDER_X, SLIDER_Y - 20, buf1);

  SDL_RenderPresent(renderer);
}

void handle_events(AppState *state, DoublePendulum *p) {
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
    case SDL_MOUSEBUTTONDOWN:
      if (event.button.button == SDL_BUTTON_LEFT) {
        int mx = event.button.x;
        int my = event.button.y;

        SDL_Rect time_slider = {SLIDER_X, SLIDER_Y - 10, SLIDER_W, 30};
        SDL_Rect len1_slider = {LEN1_SLIDER_X, LEN1_SLIDER_Y - 10, LEN_SLIDER_W,
                                30};
        SDL_Rect len2_slider = {LEN2_SLIDER_X, LEN2_SLIDER_Y - 10, LEN_SLIDER_W,
                                30};

        if (SDL_PointInRect(&(SDL_Point){mx, my}, &time_slider))
          active_slider = SLIDER_TIME;
        else if (SDL_PointInRect(&(SDL_Point){mx, my}, &len1_slider))
          active_slider = SLIDER_LEN1;
        else if (SDL_PointInRect(&(SDL_Point){mx, my}, &len2_slider))
          active_slider = SLIDER_LEN2;

        SDL_Rect reset_btn = {20, WINDOW_HEIGHT - RESET_H - 20, RESET_W,
                              RESET_H};
        if (SDL_PointInRect(&(SDL_Point){mx, my}, &reset_btn)) {
          reset_pendulum(p);
          init_trail(&state->trail1);
          init_trail(&state->trail2);
        }
      }
      break;

    case SDL_QUIT:
      state->running = 0;
      break;

    case SDL_MOUSEBUTTONUP:
      active_slider = SLIDER_NONE;
      break;

    case SDL_MOUSEMOTION:
      if (active_slider != SLIDER_NONE) {
        int mx = event.motion.x;

        double t;
        switch (active_slider) {
        case SLIDER_TIME:
          t = (double)(mx - SLIDER_X) / SLIDER_W;
          t = fmin(fmax(t, 0.0), 1.0);
          state->time_scale = 0.1 + t * 4.0;
          break;

        case SLIDER_LEN1:
          t = (double)(mx - LEN1_SLIDER_X) / LEN_SLIDER_W;
          t = fmin(fmax(t, 0.0), 1.0);
          p->length1 = slider_to_value(t, MIN_LEN, MAX_LEN);
          break;

        case SLIDER_LEN2:
          t = (double)(mx - LEN2_SLIDER_X) / LEN_SLIDER_W;
          t = fmin(fmax(t, 0.0), 1.0);
          p->length2 = slider_to_value(t, MIN_LEN, MAX_LEN);
          break;

        default:
          break;
        }
      }

      break;
    }
  }
}

int main(int argc, char *argv[]) {
  AppState *app = init_app();
  if (!app)
    return 1;

  DoublePendulum *pendulum = init_pendulum();

  Uint32 frame_start, frame_time;

  while (app->running) {
    frame_start = SDL_GetTicks();

    handle_events(app, pendulum);
    update_pendulum(pendulum, app->time_scale);
    draw_pendulum(app, app->renderer, pendulum);

    frame_time = SDL_GetTicks() - frame_start;
    if (frame_time < 1000 / FPS) {
      SDL_Delay(1000 / FPS - frame_time);
    }
  }

  free(pendulum);
  cleanup_app(app);

  return 0;
}
