FROM centos:centos6

MAINTAINER Paolo Ambrosio

RUN yum install -y java-1.8.0-openjdk-headless.x86_64 sudo

RUN yum localinstall -y https://github.com/tomakehurst/saboteur/releases/download/v0.7/saboteur-0.7-1.noarch.rpm

RUN useradd -m -s /bin/bash wiremock
ADD http://central.maven.org/maven2/com/github/tomakehurst/wiremock/1.51/wiremock-1.51-standalone.jar /usr/lib/wiremock/wiremock-standalone.jar
RUN chown -R wiremock /usr/lib/wiremock

ADD wiremock-saboteur /usr/bin/wiremock-saboteur

ENTRYPOINT /usr/bin/wiremock-saboteur

