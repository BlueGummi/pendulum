#include <SDL2/SDL.h>
#include <SDL2/SDL_ttf.h>
#include <math.h>
#include <stdio.h>

/* ALL UNITS IN HERE ARE CENTIMETERS AND ALL MASSES ARE KILOGRAMS */

#define WINDOW_FULL_WIDTH 1710
#define WINDOW_FULL_HEIGHT 1107

#define START_WINDOW_WIDTH 1400
#define START_WINDOW_HEIGHT 900
#define FPS 60
#define DT (1.0 / FPS)
#define GRAVITY 981.0
#define RESET_W 100
#define RESET_H 40
#define TRAIL_LENGTH 2000
#define SLIDER_X 150
#define SLIDER_Y(wh) (wh - 50)
#define SLIDER_W 300
#define SLIDER_H 6
#define LEN1_SLIDER_X 500
#define LEN1_SLIDER_Y(wh) (wh - 50)
#define LEN_SLIDER_W 300
#define LEN_SLIDER_H 6

#define LEN2_SLIDER_X 900
#define LEN2_SLIDER_Y(wh) (wh - 50)

#define MIN_LEN 50.0
#define MAX_LEN 300.0

#define SLIDER_ROW_HEIGHT 60
#define SLIDER_BASE_Y 50

#define MASS_REF 1.0
#define MASS_SIZE_REF 16.0

#define NO_UNIT NULL

typedef struct {
  double x[TRAIL_LENGTH];
  double y[TRAIL_LENGTH];

  double mass[TRAIL_LENGTH];
  double speed[TRAIL_LENGTH];

  int head;
  int count;
} Trail;

typedef struct {
  const char *label, *unit;
  int x;
  int row;
  int w, h;
  double min, max;
  double *value;
} Slider;

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
  double saved_time_scale;
  Trail trail1;
  Trail trail2;
  int ww, wh;

  Slider *sliders;
  int slider_count;
} AppState;

typedef struct {
  double dtheta1, domega1;
  double dtheta2, domega2;
} Deriv;

typedef struct {
  double vx, vy;
} Vec2;

static Slider *active_slider = NULL;

static inline int mass_to_thickness(double mass) {
  double t = 1.5 * sqrt(mass);
  if (t < 1)
    t = 1;
  if (t > 8)
    t = 8;
  return (int)t;
}

static inline Uint8 lerp_u8(Uint8 a, Uint8 b, double t) {
  return (Uint8)(a + t * (b - a));
}

SDL_Color mass_to_color(double mass, double min_m, double max_m) {
  double t = (mass - min_m) / (max_m - min_m);
  if (t < 0)
    t = 0;
  if (t > 1)
    t = 1;

  SDL_Color bright = {120, 255, 150, 255};
  SDL_Color dark = {20, 120, 60, 255};

  return (SDL_Color){lerp_u8(bright.r, dark.r, t), lerp_u8(bright.g, dark.g, t),
                     lerp_u8(bright.b, dark.b, t), 255};
}

static double slider_value_from_mouse(const Slider *s, int mx) {
  double t = (double)(mx - s->x) / s->w;
  if (t < 0.0)
    t = 0.0;
  if (t > 1.0)
    t = 1.0;
  return s->min + t * (s->max - s->min);
}

static double slider_t_from_value(const Slider *s) {
  return (*s->value - s->min) / (s->max - s->min);
}

static inline int slider_y(const Slider *s, int window_h) {
  return window_h - SLIDER_BASE_Y - s->row * SLIDER_ROW_HEIGHT;
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

void draw_slider(AppState *app, SDL_Renderer *r, TTF_Font *font,
                 const Slider *s) {
  SDL_Rect track = {s->x, slider_y(s, app->wh), s->w, s->h};
  SDL_SetRenderDrawColor(r, 120, 120, 120, 255);
  SDL_RenderFillRect(r, &track);

  double t = slider_t_from_value(s);
  int knob_x = s->x + (int)(t * s->w);

  SDL_Rect knob = {knob_x - 6, slider_y(s, app->wh) - 6, 12, s->h + 12};
  SDL_SetRenderDrawColor(r, 220, 220, 220, 255);
  SDL_RenderFillRect(r, &knob);

  char buf[64];
  snprintf(buf, sizeof(buf), "%s: %.2f %s", s->label, *s->value,
           s->unit ? s->unit : "(no unit)");
  draw_text(r, font, s->x, slider_y(s, app->wh) - 20, buf);
}

void slider_mouse_down(AppState *app, Slider *sliders, int count, int mx,
                       int my) {
  for (int i = 0; i < count; i++) {
    SDL_Rect hit = {sliders[i].x, slider_y(&sliders[i], app->wh) - 10,
                    sliders[i].w, sliders[i].h + 20};

    if (SDL_PointInRect(&(SDL_Point){mx, my}, &hit)) {
      active_slider = &sliders[i];
      return;
    }
  }
}

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

void init_sliders(AppState *app, DoublePendulum *p) {
  static Slider sliders[] = {
      {"Speed", NO_UNIT, 150, 0, 300, 6, 0.0, 10.0, NULL},
      {"Length 1", "cm", 500, 0, 300, 6, MIN_LEN, MAX_LEN, NULL},
      {"Length 2", "cm", 900, 0, 300, 6, MIN_LEN, MAX_LEN, NULL},
      {"Mass 1", "kg", 500, 1, 300, 6, 0.1, 10.0, NULL},
      {"Mass 2", "kg", 900, 1, 300, 6, 0.1, 10.0, NULL},
  };

  sliders[0].value = &app->time_scale;
  sliders[1].value = &p->length1;
  sliders[2].value = &p->length2;
  sliders[3].value = &p->mass1;
  sliders[4].value = &p->mass2;

  app->sliders = sliders;
  app->slider_count = sizeof(sliders) / sizeof(sliders[0]);
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
  state->saved_time_scale = NAN;
  init_trail(&state->trail1);
  init_trail(&state->trail2);

  state->ww = START_WINDOW_WIDTH;
  state->wh = START_WINDOW_HEIGHT;
  state->window = SDL_CreateWindow(
      "wahoo wee I'm crazy weehoo", SDL_WINDOWPOS_CENTERED,
      SDL_WINDOWPOS_CENTERED, state->ww, state->wh, SDL_WINDOW_SHOWN);

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

void trail_push(Trail *t, double x, double y, double mass, double speed) {
  int i = t->head;

  t->x[i] = x;
  t->y[i] = y;
  t->mass[i] = mass;
  t->speed[i] = speed;

  t->head = (i + 1) % TRAIL_LENGTH;
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

void draw_trail(SDL_Renderer *r, Trail *t, int cx, int cy, double min_mass,
                double max_mass) {
  if (t->count < 2)
    return;

  for (int i = 0; i < t->count - 1; i++) {
    int i1 = (t->head - t->count + i + TRAIL_LENGTH) % TRAIL_LENGTH;
    int i2 = (i1 + 1) % TRAIL_LENGTH;

    double fade = (double)i / (double)t->count;

    SDL_Color c = mass_to_color(t->mass[i1], min_mass, max_mass);
    c.a = (Uint8)(fade * 180);

    SDL_SetRenderDrawColor(r, c.r, c.g, c.b, c.a);

    int thickness = mass_to_thickness(t->mass[i1]);

    int x1 = cx + (int)t->x[i1];
    int y1 = cy + (int)t->y[i1];
    int x2 = cx + (int)t->x[i2];
    int y2 = cy + (int)t->y[i2];

    for (int k = -thickness / 2; k <= thickness / 2; k++) {
      SDL_RenderDrawLine(r, x1 + k, y1, x2 + k, y2);
      SDL_RenderDrawLine(r, x1, y1 + k, x2, y2 + k);
    }
  }
}

static inline int mass_to_size(double mass) {
  double s = MASS_SIZE_REF * sqrt(mass / MASS_REF);

  if (s < 6)
    s = 6;
  if (s > 40)
    s = 40;

  return (int)s;
}

void draw_pendulum(AppState *app, SDL_Renderer *renderer, DoublePendulum *p) {
  int center_x = app->ww / 2;
  int center_y = app->wh / 2;

  int x1 = center_x + (int)(p->length1 * sin(p->angle1));
  int y1 = center_y + (int)(p->length1 * cos(p->angle1));

  int x2 = x1 + (int)(p->length2 * sin(p->angle2));
  int y2 = y1 + (int)(p->length2 * cos(p->angle2));

  double rx1 = p->length1 * sin(p->angle1);
  double ry1 = p->length1 * cos(p->angle1);

  double rx2 = rx1 + p->length2 * sin(p->angle2);
  double ry2 = ry1 + p->length2 * cos(p->angle2);

  Vec2 v1 = velocity_mass1(p);
  Vec2 v2 = velocity_mass2(p);

  double mag1 = hypot(v1.vx, v1.vy);
  double mag2 = hypot(v2.vx, v2.vy);

  trail_push(&app->trail1, rx1, ry1, p->mass1, mag1);
  trail_push(&app->trail2, rx2, ry2, p->mass2, mag2);

  SDL_SetRenderDrawColor(renderer, 20, 20, 40, 255);
  SDL_RenderClear(renderer);

  draw_trail(renderer, &app->trail1, center_x, center_y, 0.1, 10.0);
  draw_trail(renderer, &app->trail2, center_x, center_y, 0.1, 10.0);

  char buf[64];

  snprintf(buf, sizeof(buf), "%.2f m/s", mag1 / 100.0);
  draw_text(renderer, app->font, x1 + 10, y1 - 10, buf);

  snprintf(buf, sizeof(buf), "%.2f m/s", mag2 / 100.0);
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

  int size1 = mass_to_size(p->mass1);
  int size2 = mass_to_size(p->mass2);

  SDL_Rect mass1_rect = {x1 - size1 / 2, y1 - size1 / 2, size1, size1};

  SDL_Rect mass2_rect = {x2 - size2 / 2, y2 - size2 / 2, size2, size2};

  SDL_SetRenderDrawColor(renderer, 50, 200, 100, 255);
  SDL_RenderFillRect(renderer, &mass1_rect);

  SDL_SetRenderDrawColor(renderer, 100, 255, 150, 255);
  SDL_RenderFillRect(renderer, &mass2_rect);

  SDL_Rect reset_btn = {20, app->wh - RESET_H - 20, RESET_W, RESET_H};

  SDL_SetRenderDrawColor(renderer, 60, 60, 60, 255);
  SDL_RenderFillRect(renderer, &reset_btn);

  SDL_SetRenderDrawColor(renderer, 200, 200, 200, 255);
  SDL_RenderDrawRect(renderer, &reset_btn);

  for (int i = 0; i < app->slider_count; i++) {
    draw_slider(app, renderer, app->font, &app->sliders[i]);
  }

  SDL_RenderPresent(renderer);
}

void handle_events(AppState *state, DoublePendulum *p) {
  SDL_Event event;
  SDL_KeyboardEvent *key;
  while (SDL_PollEvent(&event)) {
    switch (event.type) {
    case SDL_MOUSEBUTTONDOWN:
      if (event.button.button == SDL_BUTTON_LEFT) {
        int mx = event.button.x;
        int my = event.button.y;

        slider_mouse_down(state, state->sliders, state->slider_count,
                          event.button.x, event.button.y);

        SDL_Rect reset_btn = {20, state->wh - RESET_H - 20, RESET_W, RESET_H};
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

    case SDL_KEYDOWN:
      key = &event.key;
      static int fullscreen = 0;

      switch (key->keysym.sym) {
      case SDLK_f:
        fullscreen = !fullscreen;
        if (fullscreen) {
          SDL_SetWindowSize(state->window, WINDOW_FULL_WIDTH,
                            WINDOW_FULL_HEIGHT);
          state->ww = WINDOW_FULL_WIDTH;
          state->wh = WINDOW_FULL_HEIGHT;
        } else {
          SDL_SetWindowSize(state->window, START_WINDOW_WIDTH,
                            START_WINDOW_HEIGHT);
          state->ww = START_WINDOW_WIDTH;
          state->wh = START_WINDOW_HEIGHT;
        }

        SDL_SetWindowFullscreen(state->window,
                                fullscreen ? SDL_WINDOW_FULLSCREEN : 0);
        break;
      case SDLK_SPACE:
        if (isnan(state->saved_time_scale)) {
          state->saved_time_scale = state->time_scale;
          state->time_scale = 0.0;
        } else {
          state->time_scale = state->saved_time_scale;
          state->saved_time_scale = NAN;
        }
        break;
      case SDLK_r:
        p->mass1 = 1.0;
        p->mass2 = 1.0;
        p->length1 = 150.0;
        p->length2 = 150.0;
        break;
      }

      break;

    case SDL_MOUSEBUTTONUP:
      active_slider = NULL;
      break;

    case SDL_MOUSEMOTION:
      if (active_slider) {
        *active_slider->value =
            slider_value_from_mouse(active_slider, event.motion.x);
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
  init_sliders(app, pendulum);

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
