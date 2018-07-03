#!/usr/bin/env python
'''
A module for reading in Blender Scene Descriptions from yaml.
'''

import os
import sys
#pylint: disable=import-error
import bpy

CURRENT_DIRECTORY = os.getcwd()
if not CURRENT_DIRECTORY in sys.path:
    sys.path.append(CURRENT_DIRECTORY)

#pylint: disable=wrong-import-position
import blender_utils
import utils

# TODO: Setup similar to load scene:
#   load cameras, lights, and other scene objects
#     camera direction and position
#     sun lights, point lights, rays, cones, etc,

def add_light(light):
    pass

def add_camera(camera):
    pass

def set_render_settings(settings):
    '''
    Settings for rendering. See
    https://docs.blender.org/api/current/bpy.types.RenderSettings.html
    for details.
    '''
    r_set = bpy.types.RenderSettings()
    ALL_SETTINGS = [
        'alpha_mode',
        'antialiasing_samples',
        'border_max_x',
        'border_max_y',
        'border_min_x',
        'border_min_y',
        'display_mode',
        'dither_intensity',
        'edge_color',
        'edge_threshold',
        'engine',
        'field_order',
        'file_extension',
        'filepath',
        'filter_size',
        'fps',
        'fps_base',
        'frame_map_new',
        'frame_map_old',
        'has_multiple_engines',
        'is_movie_format',
        'line_thickness',
        'line_thickness_mode',
        'motion_blur_samples',
        'motion_blur_shutter',
        'motion_blur_shutter_curve',
        'octree_resolution',
        'pixel_aspect_x',
        'pixel_aspect_y',
        'pixel_filter_type',
        'preview_start_resolution',
        'raytrace_method',
        'resolution_percentage',
        'resolution_x',
        'resolution_y',
        'sequencer_gl_preview',
        'simplify_ao_sss',
        'simplify_child_particles',
        'simplify_child_particles_render',
        'simplify_shadow_samples',
        'simplify_subdivision',
        'simplify_subdivision_render',
        'stamp_background',
        'stamp_font_size',
        'stamp_foreground',
        'stamp_note_text',
        'threads',
        'threads_mode',
        'tile_x',
        'tile_y',
        'use_border',
        'use_compositing',
        'use_crop_to_border',
        'use_edge_enhance',
        'use_envmaps',
        'use_file_extension',
        'use_full_sample',
        'use_local_coords',
        'use_motion_blur',
        'use_multiview',
        'use_overwrite',
        'ues_persistent_data',
        'use_raytrace',
        'use_render_cache',
        'use_shading_nodes',
        'use_shadows',
        'use_simplify',
        'use_single_layer',
        'use_sss',
        'use_stamp',
        'use_stamp_date',
        'use_stamp_lens',
        'use_stamp_memory',
        'use_stamp_render_time',
        'use_textures',
        'use_world_space_shading'
    ]
    for attr in ALL_SETTINGS:
        if attr in settings:
            setattr(r_set, attr, settings[attr])
    bpy.context.scene.render = r_set
    if 'ffmpeg' in settings:
        set_ffmpeg_settings(settings['ffmpeg'])
    if 'image_format_settings' in settings:
        set_image_format_settings(settings['image_format_settings'])

def set_ffmpeg_settings(settings):
    '''
    https://docs.blender.org/api/current/bpy.types.FFmpegSettings.html
    '''
    FFMPEG_SETTINGS = [
        'audio_bitrate',
        'audio_channels',
        'audio_codec',
        'audio_mixrate',
        'audio_volume',
        'buffersize',
        'codec',
        'constant_rate_factor',
        'ffmpeg_preset',
        'format',
        'gopsize',
        'max_b_frames',
        'maxrate',
        'minrate',
        'muxrate',
        'packetsize',
        'use_autosplit',
        'use_lossless_output',
        'use_max_b_frames',
        'video_bitrate'
    ]
    f_set = bpy.types.FFmpegSettings()
    for attr in FFMPEG_SETTINGS:
        if attr in settings:
            setattr(f_set, attr, settings[attr])
    bpy.context.scene.render.ffmpeg = f_set

def set_image_format_settings(settings):
    '''
    https://docs.blender.org/api/current/bpy.types.ImageFormatSettings.html
    '''
    IMAGE_SETTINGS = [
        'cineon_black',
        'cineon_gamma',
        'cineon_white',
        'color_depth',
        'color_mode',
        'compression',
        'display_settings',
        'exr_codec',
        'file_format',
        'quality',
        'stereo_3d_format',
        'tiff_codec',
        'use_cineon_log',
        'use_preview',
        'use_zbuffer',
        'view_settings',
        'views_format'
    ]
    i_set = bpy.types.ImageFormatSettings()
    for attr in IMAGE_SETTINGS:
        if attr in settings:
            setattr(i_set, attr, settings[attr])
    bpy.context.scene.render.image_format_settings = i_set


def set_light_settings(settings):
    '''
    Settings for world lighting. See
    https://docs.blender.org/api/current/bpy.types.WorldLighting.html
    for details.
    '''
    w_set = bpy.types.WorldLight()
    ALL_SETTINGS = [
        'adapt_to_speed',
        'ao_blend_type',
        'ao_factor',
        'bias',
        'correction',
        'distance',
        'environment_color',
        'environment_energy',
        'error_threshold',
        'falloff_strength',
        'gather_method',
        'indirect_bounces',
        'indirect_factor',
        'passes',
        'sample_method',
        'samples',  
        'threshold',
        'use_ambient_occlusion',
        'use_cache',
        'use_environment_light',
        'use_indirect_light',
        'use_falloff'
    ]
    for attr in ALL_SETTINGS:
        if attr in settings:
            setattr(w_set, attr, settings[attr])
    if not bpy.data.worlds:
        bpy.ops.world.new()
    bpy.data.worlds['World'].light_settings = w_set
    pass


def add_blender_scene(scene):
    for light in scene.get('lights', []):
        add_light(light)
    if 'camera' in scene:
        add_camera(scene['camera'])
    set_render_settings(scene.get('render', {}))
    set_light_settings(scene.get('light_settings', {}))
    # TODO: add various other objects to the scene
