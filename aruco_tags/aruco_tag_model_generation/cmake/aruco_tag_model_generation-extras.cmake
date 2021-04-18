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
endfunction(generate_aruco_tag_model)

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
endfunction(generate_aruco_tag_models)
