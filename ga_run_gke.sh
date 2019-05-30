#!/bin/bash

gcloud container clusters create drones-cluster --zone=us-central1-a --machine-type=n1-highcpu-2 --num-nodes=4
gcloud container clusters get-credentials drones-cluster

kubectl apply -f ga_evaluator_deployment.yaml
kubectl apply -f ga_arbitrator_deployment.yaml

