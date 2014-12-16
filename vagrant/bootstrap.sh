#!/bin/sh

yum install -y java-1.8.0-openjdk-headless.x86_64 wget sudo

yum localinstall -y https://github.com/tomakehurst/saboteur/releases/download/v0.7/saboteur-0.7-1.noarch.rpm

useradd -m -s /bin/bash wiremock
mkdir -p /usr/lib/wiremock
wget -q http://central.maven.org/maven2/com/github/tomakehurst/wiremock/1.51/wiremock-1.51-standalone.jar -O /usr/lib/wiremock/wiremock-standalone.jar
mkdir /var/log/wiremock
chown -R wiremock:wiremock /usr/lib/wiremock /var/log/wiremock

