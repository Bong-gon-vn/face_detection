cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(s3_cam)

# Generate ESP-DL model assembly during CMake configuration.
# PlatformIO then compiles these generated files with the application.

idf_build_get_property(face_python PYTHON)
idf_build_get_property(face_build_dir BUILD_DIR)

idf_component_get_property(
    face_dl_dir espressif__esp-dl COMPONENT_DIR
)
idf_component_get_property(
    face_detect_dir espressif__human_face_detect COMPONENT_DIR
)
idf_component_get_property(
    face_recognition_dir espressif__human_face_recognition COMPONENT_DIR
)

function(prepare_face_model model_name)
    set(model_binary
        "${face_build_dir}/espdl_models/${model_name}.espdl"
    )
    set(model_assembly
        "${face_build_dir}/${model_name}.espdl.S"
    )

    file(MAKE_DIRECTORY "${face_build_dir}/espdl_models")

    execute_process(
        COMMAND
            "${face_python}"
            "${face_dl_dir}/fbs_loader/pack_espdl_models.py"
            --model_path ${ARGN}
            --out_file "${model_binary}"
        RESULT_VARIABLE pack_result
        OUTPUT_VARIABLE pack_output
        ERROR_VARIABLE pack_error
    )

    if(NOT pack_result EQUAL 0)
        message(FATAL_ERROR
            "Cannot pack ${model_name}:\n"
            "${pack_output}\n${pack_error}"
        )
    endif()

    execute_process(
        COMMAND
            "${CMAKE_COMMAND}"
            "-DDATA_FILE=${model_binary}"
            "-DSOURCE_FILE=${model_assembly}"
            "-DFILE_TYPE=BINARY"
            -P
            "${face_dl_dir}/fbs_loader/cmake/data_file_embed_asm_aligned.cmake"
        RESULT_VARIABLE embed_result
        OUTPUT_VARIABLE embed_output
        ERROR_VARIABLE embed_error
    )

    if(NOT embed_result EQUAL 0)
        message(FATAL_ERROR
            "Cannot embed ${model_name}:\n"
            "${embed_output}\n${embed_error}"
        )
    endif()
endfunction()

# Face detection: MSR + MNP.
if(CONFIG_HUMAN_FACE_DETECT_MODEL_IN_FLASH_RODATA)
    prepare_face_model(
        human_face_detect
        "${face_detect_dir}/models/s3/human_face_detect_msr_s8_v1.espdl"
        "${face_detect_dir}/models/s3/human_face_detect_mnp_s8_v1.espdl"
    )
endif()

# Face embedding: MobileFaceNet.
if(CONFIG_HUMAN_FACE_FEAT_MODEL_IN_FLASH_RODATA)
    prepare_face_model(
        human_face_feat
        "${face_recognition_dir}/models/s3/human_face_feat_mfn_s8_v1.espdl"
    )
endif()