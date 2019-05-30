#!/bin/bash

echo Using PROJECT_ID: $PROJECT_ID

mkdir docker_data
cp bazel-bin/main docker_data/
cp data/*.in docker_data/

# Arbitrator image:
echo Building ga_arbitrator-image
docker build -t ga_arbitrator-image -f ga_arbitrator_dockerfile .
docker tag ga_arbitrator-image gcr.io/${PROJECT_ID}/ga_arbitrator-image:latest
docker push gcr.io/${PROJECT_ID}/ga_arbitrator-image:latest

# Evaluator image:
echo Building ga_evaluator-image
docker build -t ga_evaluator-image -f ga_evaluator_dockerfile .
docker tag ga_evaluator-image gcr.io/${PROJECT_ID}/ga_evaluator-image:latest
docker push gcr.io/${PROJECT_ID}/ga_evaluator-image:latest

rm -rf docker_data
