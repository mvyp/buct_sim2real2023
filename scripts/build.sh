#!/bin/bash
CLIENT_IMAGE=${CLIENT_IMAGE:-client-custom:test}
docker build . -t $CLIENT_IMAGE