Experiment with Akka Cluster Management

Run it on Kubernetes (Minikube):
```sh
eval $(minikube docker-env)
sbt docker:publishLocal
kubectl create -f kubernetes/application.yaml
curl $(minikube service frontend-service --url)/greet/World
```
