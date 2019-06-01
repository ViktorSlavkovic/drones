#!/bin/bash

# Add Kubernetes Engine Admin role first!
gcloud beta container clusters create drones-cluster \
  --addons=Istio --istio-config=auth=MTLS_PERMISSIVE  \
  --cluster-version=1.12.7-gke.10 \
  --zone=us-central1-a --machine-type=n1-highcpu-16 --num-nodes=6

gcloud container clusters get-credentials drones-cluster --zone=us-central1-a

kubectl label namespace default istio-injection=enabled

kubectl apply -f ga_evaluator_deployment.yaml
kubectl apply -f ga_arbitrator_deployment.yaml

