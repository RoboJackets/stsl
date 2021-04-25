# Copyright 2021 RoboJackets
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

function(generate_aruco_tag_model)
    set(oneValueArgs ID RESOLUTION SIZE DESTINATION)
    cmake_parse_arguments(_GEN_TAG_MODEL "" "${oneValueArgs}" "" ${ARGN})
    set(generated_model_files "")
    list(APPEND generated_model_files "${_GEN_TAG_MODEL_DESTINATION}/aruco_tag_${_GEN_TAG_MODEL_ID}/model.config")
    list(APPEND generated_model_files "${_GEN_TAG_MODEL_DESTINATION}/aruco_tag_${_GEN_TAG_MODEL_ID}/model.sdf")
    list(APPEND generated_model_files "${_GEN_TAG_MODEL_DESTINATION}/aruco_tag_${_GEN_TAG_MODEL_ID}/materials/scripts/aruco_tag_${_GEN_TAG_MODEL_ID}.material")
    list(APPEND generated_model_files "${_GEN_TAG_MODEL_DESTINATION}/aruco_tag_${_GEN_TAG_MODEL_ID}/materials/textures/ArUcoTag${_GEN_TAG_MODEL_ID}.jpg")
    set(_model_dir_name "aruco_tag_${_GEN_TAG_MODEL_ID}")
    add_custom_command(
        OUTPUT ${generated_model_files}
        COMMAND aruco_tag_model_generation::tag_model_generator ${_GEN_TAG_MODEL_ID} ${_GEN_TAG_MODEL_RESOLUTION} ${_GEN_TAG_MODEL_SIZE} "${_GEN_TAG_MODEL_DESTINATION}"
        DEPENDS aruco_tag_model_generation::tag_model_generator
        COMMENT "Generating tag model ID ${_GEN_TAG_MODEL_ID}"
    )
    set(GENERATED_TAG_MODEL_FILES ${generated_model_files} PARENT_SCOPE)
endfunction()

function(generate_aruco_tag_models)
    set(oneValueArgs IDS_START IDS_STOP RESOLUTION SIZE DESTINATION)
    cmake_parse_arguments(_GEN_TAG_MODELS "" "${oneValueArgs}" "" ${ARGN})
    if(DEFINED _GEN_TAG_MODELS_KEYWORDS_MISSING_VALUES)
        message(ERROR "Please specify all arguments to generate_aruco_tag_models(). You were missing these: ${_GEN_TAG_MODELS_KEYWORDS_MISSING_VALUES}")
    endif()
    set(generated_files "")
    foreach(tag_id RANGE ${_GEN_TAG_MODELS_IDS_START} ${_GEN_TAG_MODELS_IDS_STOP})
        generate_aruco_tag_model(
            ID ${tag_id}
            RESOLUTION ${_GEN_TAG_MODELS_RESOLUTION}
            SIZE ${_GEN_TAG_MODELS_SIZE}
            DESTINATION ${_GEN_TAG_MODELS_DESTINATION})
        list(APPEND generated_files ${GENERATED_TAG_MODEL_FILES})
    endforeach()
    message(STATUS "All generated model files: ${generated_files}")
    add_custom_target(
        ${PROJECT_NAME}_GENERATE_ARUCO_TAG_MODELS
        ALL
        DEPENDS ${generated_files}
    )
endfunction()
