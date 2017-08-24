#!/bin/sh

glad --profile="core" --api="gl=3.3" --generator="c" --spec="gl" \
     --out-path=src/glad/ \
     --extensions="GL_ARB_vertex_array_object,GL_ARB_texture_storage"
