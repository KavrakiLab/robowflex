#!/usr/bin/env python
'''A module for reading in Blender Scene Descriptions from yaml.

'''

import os
import sys
# pylint: disable=import-error
import bpy

CURRENT_DIRECTORY = os.getcwd()
if not CURRENT_DIRECTORY in sys.path:
    sys.path.append(CURRENT_DIRECTORY)

# pylint: disable=wrong-import-position
import blender_utils
import utils

COMMON_LIGHT_SETTINGS = [
    'shadow_adaptive_threshold',
    'shadow_buffer_bias',
    'shadow_buffer_bleed_bias',
    'shadow_buffer_clip_end',
    'shadow_buffer_clip_start',
    'shadow_buffer_samples',
    'shadow_buffer_size',
    'shadow_buffer_soft',
    'shadow_buffer_type',
    'shadow_color',
    'shadow_filter_type',
    'shadow_method',
    'shadow_ray_sample_method',
    'shadow_ray_samples',
    'shadow_sample_buffers',
    'shadow_soft_size',
    'use_auto_clip_end',
    'use_auto_clip_start',
    'use_only_shadow',
    'use_shadow',
    'use_shadow_layer'
] # yapf: disable


def add_point_light(light):
    '''https://docs.blender.org/api/current/bpy.types.PointLamp.html

    '''
    bpy.ops.object.lamp_add(type = 'POINT', location = blender_utils.pose_to_vec(light['pose']))
    ALL_SETTINGS = COMMON_LIGHT_SETTINGS + [
        'compression_threshold',
        'constant_coefficient',
        'falloff_curve',
        'falloff_type',
        'ge_shadow_buffer_type',
        'linear_attenuation',
        'linear_coefficient',
        'quadratic_attenuation',
        'quadratic_coefficient',
        'use_sphere'
    ] # yapf: disable
    for attr in ALL_SETTINGS:
        if attr in light:
            setattr(bpy.context.active_object.data, attr, light[attr])
    return


def add_sun_light(light):
    '''https://docs.blender.org/api/current/bpy.types.SunLamp.html

    '''
    bpy.ops.object.lamp_add(type = 'SUN', location = blender_utils.pose_to_vec(light['pose']))
    ALL_SETTINGS = COMMON_LIGHT_SETTINGS + [
        'compression_threshold',
        'ge_shadow_buffer_type',
        'shadow_frustum_size',
        'show_shadow_box',
    ]
    for attr in ALL_SETTINGS:
        if attr in light:
            setattr(bpy.context.active_object.data, attr, light[attr])
    return


def add_spot_light(light):
    '''https://docs.blender.org/api/current/bpy.types.SpotLamp.html

    '''
    bpy.ops.object.lamp_add(type = 'SPOT', location = blender_utils.pose_to_vec(light['pose']))
    bpy.context.active_object.rotation_mode = 'QUATERNION'
    bpy.context.active_object.rotation_quaternion = blender_utils.pose_to_quat(light['pose'])
    ALL_SETTINGS = COMMON_LIGHT_SETTINGS + [
        'compression_threshold',
        'constant_coefficient',
        'falloff_curve',
        'falloff_type',
        'ge_shadow_buffer_type',
        'halo_intensity',
        'halo_step',
        'linear_attenuation',
        'linear_coefficient',
        'quadratic_attenuation',
        'quadratic_coefficient',
        'show_cone',
        'spot_blend',
        'spot_size',
        'use_halo',
        'use_sphere',
        'use_square',
    ] # yapf: disable
    for attr in ALL_SETTINGS:
        if attr in light:
            setattr(bpy.context.active_object.data, attr, light[attr])
    return


def add_hemi_light(light):
    '''https://docs.blender.org/api/current/bpy.types.HemiLamp.html

    '''
    bpy.ops.object.lamp_add(type = 'HEMI', location = blender_utils.pose_to_vec(light['pose']))
    return


def add_area_light(light):
    '''https://docs.blender.org/api/current/bpy.types.AreaLamp.html

    '''
    bpy.ops.object.lamp_add(type = 'AREA', location = blender_utils.pose_to_vec(light['pose']))
    bpy.context.active_object.rotation_mode = 'QUATERNION'
    bpy.context.active_object.rotation_quaternion = blender_utils.pose_to_quat(light['pose'])
    ALL_SETTINGS = COMMON_LIGHT_SETTINGS + [
        'compression_threshold',
        'gamma',
        'ge_shadow_buffer_type',
        'shape',
        'size',
        'size_y',
        'use_dither',
        'use_jitter',
        'use_umbra'
    ] # yapf: disable
    for attr in ALL_SETTINGS:
        if attr in light:
            setattr(bpy.context.active_object.data, attr, light[attr])
    return


LIGHT_MAP = {
    'point': add_point_light,
    'sun': add_sun_light,
    'spot': add_spot_light,
    'hemi': add_hemi_light,
    'area': add_area_light,
}


def add_light(light):
    LIGHT_MAP[light['type'].lower()](light)
    # Set the common light settings:
    ALL_SETTINGS = [
        'color',
        'distance',
        'energy',
        'use_diffuse',
        'use_negative',
        'use_nodes',
        'use_own_layer',
        'use_specular'
    ] # yapf: disable
    for attr in ALL_SETTINGS:
        if attr in light:
            setattr(bpy.context_active_object.data, attr, light[attr])
    return


def add_camera(camera):
    blender_utils.add_camera(blender_utils.pose_to_vec(camera['pose']), blender_utils.pose_to_quat(camera['pose']))


def set_render_settings(settings):
    '''Settings for rendering. See https://docs.blender.org/api/current/bpy.types.RenderSettings.html for details.

    '''
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
    ] # yapf: disable

    for attr in ALL_SETTINGS:
        if attr in settings:
            setattr(bpy.context.scene.render, attr, settings[attr])
    if 'ffmpeg' in settings:
        set_ffmpeg_settings(settings['ffmpeg'])
    if 'image_format_settings' in settings:
        set_image_format_settings(settings['image_format_settings'])


def set_ffmpeg_settings(settings):
    '''https://docs.blender.org/api/current/bpy.types.FFmpegSettings.html

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
    ] # yapf: disable

    for attr in FFMPEG_SETTINGS:
        if attr in settings:
            setattr(bpy.context.scene.render.ffmpeg, attr, settings[attr])


def set_image_format_settings(settings):
    '''https://docs.blender.org/api/current/bpy.types.ImageFormatSettings.html

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
    ] # yapf: disable
    for attr in IMAGE_SETTINGS:
        if attr in settings:
            setattr(bpy.context.scene.render.image_format_settings, attr, settings[attr])


def set_light_settings(settings):
    '''Settings for world lighting. See https://docs.blender.org/api/current/bpy.types.WorldLighting.html for details.

    '''
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
    ] # yapf: disable

    if not bpy.data.worlds:
        bpy.ops.world.new()
    for attr in ALL_SETTINGS:
        if attr in settings:
            setattr(bpy.data.worlds['World'].light_settings, attr, settings[attr])


def set_world_settings(settings):
    '''Settings for the world. See https://docs.blender.org/api/current/bpy.types.World.html for details.
    '''
    ALL_SETTINGS = [
        'active_texture_index',
        'ambient_color',
        'color_range',
        'exposure',
        'horizon_color',
        'use_sky_blend',
        'use_sky_paper',
        'use_sky_real',
        'zenith_color'
    ] # yapf: disable
    if not bpy.data.worlds:
        bpy.ops.world.new()
    for attr in ALL_SETTINGS:
        if attr in settings:
            setattr(bpy.data.worlds['World'], attr, settings[attr])


def add_blender_scene(scenefile):
    scene = utils.read_yaml_data(scenefile)
    for light in scene.get('lights', []):
        add_light(light)
    if 'camera' in scene:
        add_camera(scene['camera'])
    set_render_settings(scene.get('render', {}))
    set_light_settings(scene.get('light_settings', {}))
    set_world_settings(scene.get('world_settings', {}))
    # TODO: add various other objects to the scene
