Experiment with location transparency with Akka

The purpose of this project is to showcase how easy it is to go from local
to distributed actors in Akka. In no way it is intended as a guide on how to
write good Scala or Akka code.

It has no tests, but on the other side there isn't much code either!

Run the application locally (each command on different terminal):
```sh
REMOTE_PORT=2551 sbt backend/run
REMOTE_PORT=2552 sbt backend/run
REMOTE_PORT=2553 BACKEND_NODES=127.0.0.1:2551,127.0.0.1:2552 sbt frontend/run
curl http://localhost:9000/greet/World
```

Run it on Kubernetes (Minikube):
```sh
eval $(minikube docker-env)
sbt docker:publishLocal
kubectl create -f kubernetes/backend-service.yaml
kubectl create -f kubernetes/backend-deployment.yaml
kubectl create -f kubernetes/frontend-service.yaml
kubectl create -f kubernetes/frontend-deployment.yaml
curl $(minikube service frontend-service --url)/greet/World
```
