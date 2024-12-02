#include "Speedo_demo.h"
#include <stdio.h>
#include "fonts/montserrat_bold_28.h"
#include "fonts/montserrat_bold_80.h"
#include "fonts/montserrat_bold_120.h"

// Global components
lv_obj_t *scr; // Screen background
lv_obj_t *rpm_arc; // Moving arc
lv_obj_t *rpm_label; // Digital RPM
lv_obj_t *mph_label; // Digital MPH
lv_obj_t *gear_label; // Digital gear

// Global styles
static lv_style_t style_unit_text;

// Color palette
lv_color_t palette_amber = LV_COLOR_MAKE(250, 140, 0);
lv_color_t palette_black = LV_COLOR_MAKE(0, 0, 0);
lv_color_t palette_red = LV_COLOR_MAKE(255, 0, 0);
lv_color_t palette_white = LV_COLOR_MAKE(255, 255, 255);

// Attributes
int rpm = 0;

const int dimension = 480;
const int rpm_max = 8000;
const int rpm_redline = 6600;
const int rpm_arc_width = 24;
const int rpm_line_width = 4;
const int rpm_arc_size = dimension - (rpm_line_width * 12);
const int arc_start = 120;
const int arc_end = 344;
const int rpm_freq = 200;
const float marker_gap = ((float)arc_end - arc_start) / (rpm_max / 1000);

// Styles

void make_styles(void) {
    lv_style_init(&style_unit_text);
    lv_style_set_text_font(&style_unit_text, &montserrat_bold_28);
    lv_style_set_text_color(&style_unit_text, palette_amber);
}

// Update values

void arc_anim_cb(void *arc, int32_t value) {
  lv_arc_set_value((lv_obj_t *)arc, value);
}

void update_speed(int new_speed) {
    static char mph_text[8];
    snprintf(mph_text, sizeof(mph_text), "%d", new_speed);
    lv_label_set_text(mph_label, mph_text); 
}

void update_gear(int new_gear) {
    static char gear_text[8];
    snprintf(gear_text, sizeof(gear_text), "%d", new_gear);
    lv_label_set_text(mph_label, gear_text); 
}

void update_rpm(int new_rpm) {
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, rpm_arc);
    lv_anim_set_values(&a, lv_arc_get_value(rpm_arc), new_rpm);
    lv_anim_set_time(&a, rpm_freq);
    lv_anim_set_exec_cb(&a, arc_anim_cb);
    lv_anim_set_path_cb(&a, lv_anim_path_linear); 
    lv_anim_start(&a);

    if (new_rpm >= rpm_redline) {
      lv_obj_set_style_arc_color(rpm_arc, palette_red,  LV_PART_INDICATOR);
    } else {
      lv_obj_set_style_arc_color(rpm_arc, palette_amber,  LV_PART_INDICATOR);
    }

    static char rpm_text[8];
    snprintf(rpm_text, sizeof(rpm_text), "%d", new_rpm);
    lv_label_set_text(rpm_label, rpm_text);

    rpm = new_rpm;
}


// Maker UI Elements

void make_rpm_arc(void) {
    rpm_arc = lv_arc_create(scr);

    lv_obj_set_size(rpm_arc, rpm_arc_size, rpm_arc_size);
    lv_obj_align(rpm_arc, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_bg_angles(rpm_arc, arc_start + 2, arc_end - 1);

    lv_obj_set_style_arc_color(rpm_arc, palette_black,  LV_PART_MAIN);
    lv_obj_set_style_arc_color(rpm_arc, palette_amber,  LV_PART_INDICATOR);
    lv_obj_set_style_arc_rounded(rpm_arc, false, LV_PART_INDICATOR);
    lv_obj_set_style_arc_width(rpm_arc, rpm_arc_width, LV_PART_INDICATOR);

    lv_obj_remove_style(rpm_arc, NULL, LV_PART_KNOB); 

    lv_arc_set_range(rpm_arc, 0, rpm_max);
    lv_arc_set_value(rpm_arc, rpm_max);
}

void position_markers(lv_obj_t *marker, int position) {
    lv_obj_set_size(marker, rpm_arc_size + (rpm_line_width * 4), rpm_arc_size + (rpm_line_width * 4));
    lv_obj_align(marker, LV_ALIGN_CENTER, 0, 0);
    
    lv_obj_set_style_arc_width(marker, rpm_arc_width + (rpm_line_width * 4), LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(marker, false, LV_PART_MAIN);
    lv_obj_set_style_arc_color(marker, palette_white, LV_PART_MAIN);

    lv_arc_set_bg_angles(marker, arc_start + (position * marker_gap), arc_start + (position * marker_gap) + 1);

    lv_obj_remove_style(marker, NULL, LV_PART_KNOB); 
    lv_obj_remove_style(marker, NULL, LV_PART_INDICATOR);    
}

void make_rpm_border(void) {
    lv_obj_t *rpm_arc_border = lv_arc_create(scr);

    lv_obj_set_size(rpm_arc_border, rpm_arc_size - (rpm_arc_width * 2) - (rpm_line_width * 2), rpm_arc_size - (rpm_arc_width * 2) - (rpm_line_width * 2));
    lv_obj_align(rpm_arc_border, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_bg_angles(rpm_arc_border, arc_start, arc_end);

    lv_obj_set_style_arc_width(rpm_arc_border, rpm_line_width, LV_PART_MAIN); 
    lv_obj_set_style_arc_rounded(rpm_arc_border, false, LV_PART_MAIN);
    lv_obj_set_style_arc_color(rpm_arc_border, palette_white, LV_PART_MAIN);

    lv_obj_remove_style(rpm_arc_border, NULL, LV_PART_KNOB); 
    lv_obj_remove_style(rpm_arc_border, NULL, LV_PART_INDICATOR);

    int rpm_marker_count = (rpm_max / 1000) + 1;
    lv_obj_t *rpm_markers[rpm_marker_count];

    for (int i = 0; i < rpm_marker_count; i++) {
        rpm_markers[i] = lv_arc_create(scr);
        position_markers(rpm_markers[i], i);
    }
}

void make_rpm_redline(void) {
    lv_obj_t *rpm_redline_marker = lv_arc_create(scr);

    lv_obj_set_size(rpm_redline_marker, dimension, dimension);
    lv_obj_align(rpm_redline_marker, LV_ALIGN_CENTER, 0, 0);
    lv_arc_set_bg_angles(rpm_redline_marker, arc_start + ((arc_end - arc_start) * ((float)rpm_redline / rpm_max)), arc_end);

    lv_obj_set_style_arc_color(rpm_redline_marker, palette_red, LV_PART_MAIN);
    lv_obj_set_style_arc_rounded(rpm_redline_marker, false, LV_PART_MAIN);
    lv_obj_set_style_arc_width(rpm_redline_marker, rpm_line_width * 2, LV_PART_MAIN);
    
    lv_obj_remove_style(rpm_redline_marker, NULL, LV_PART_KNOB); 
    lv_obj_remove_style(rpm_redline_marker, NULL, LV_PART_INDICATOR); 
}

void make_rpm_digital(void) {
    static lv_style_t style_rpm_text;
    lv_style_init(&style_rpm_text);
    lv_style_set_text_font(&style_rpm_text, &lv_font_montserrat_40);
    lv_style_set_text_color(&style_rpm_text, palette_white);

    rpm_label = lv_label_create(scr);
    lv_label_set_text(rpm_label, "2500");
    lv_obj_add_style(rpm_label, &style_rpm_text, 0);
    lv_obj_align(rpm_label, LV_ALIGN_LEFT_MID, 360, -10);

    lv_obj_t *rpm_unit_label = lv_label_create(scr);
    lv_label_set_text(rpm_unit_label, "rpm");
    lv_obj_add_style(rpm_unit_label, &style_unit_text, 0);
    lv_obj_align(rpm_unit_label, LV_ALIGN_LEFT_MID, 360, 20);

}

void make_speed_digital(void) {
    static lv_style_t style_speed_text;
    lv_style_init(&style_speed_text);
    lv_style_set_text_font(&style_speed_text, &montserrat_bold_120);
    lv_style_set_text_color(&style_speed_text, palette_white);

    mph_label = lv_label_create(scr);
    lv_label_set_text(mph_label, "85");
    lv_obj_add_style(mph_label, &style_speed_text, 0);
    lv_obj_align(mph_label, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *mph_unit_label = lv_label_create(scr);
    lv_label_set_text(mph_unit_label, "mph");
    lv_obj_add_style(mph_unit_label, &style_unit_text, 0);
    lv_obj_align(mph_unit_label, LV_ALIGN_CENTER, 40, 60);
}

void make_gear(void) { 
    static lv_color_t canvas_buf[LV_CANVAS_BUF_SIZE_TRUE_COLOR(120, 120)];
    lv_obj_t *canvas = lv_canvas_create(scr);
    lv_canvas_set_buffer(canvas, canvas_buf, 120, 120, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(canvas, LV_ALIGN_CENTER, 0, 180);
    lv_obj_set_style_border_width(canvas, 0, 0);
    lv_obj_set_style_bg_opa(canvas, LV_OPA_0, 0);

    lv_point_t points[] = {
        {15, 15},  
        {105, 15},
        {120, 120},
        {0, 120},
    };

    lv_draw_rect_dsc_t gear_bg_style;
    lv_draw_rect_dsc_init(&gear_bg_style);
    gear_bg_style.bg_color = lv_color_make(45, 45, 45);
    gear_bg_style.bg_opa = LV_OPA_COVER; 
    gear_bg_style.border_width = 0; 

   lv_canvas_draw_polygon(canvas, points, 4, &gear_bg_style);

    gear_label = lv_label_create(scr);

    static lv_style_t style_gear_text;
    lv_style_init(&style_gear_text);
    lv_style_set_text_font(&style_gear_text, &montserrat_bold_80);
    lv_style_set_text_color(&style_gear_text, palette_amber);

    lv_label_set_text(gear_label, "N");
    lv_obj_add_style(gear_label, &style_gear_text, 0);
    lv_obj_align(gear_label, LV_ALIGN_CENTER, 0, 190);
}

void Build_UI(void){
    scr = lv_scr_act();    
    lv_obj_set_style_bg_color(scr, palette_black, 0);

    // Common styles
    make_styles();

    // RPM stuff
    make_rpm_arc();
    make_rpm_border();
    make_rpm_redline();
    make_rpm_digital();

    // Fast boi stuff
    make_speed_digital();
    make_gear();

    update_rpm(3500);
}