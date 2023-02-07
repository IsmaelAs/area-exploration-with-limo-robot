FROM ros:noetic

RUN apt update
RUN apt install curl -y

# install node:18.13
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.1/install.sh | bash
RUN . ~/.nvm/nvm.sh && . ~/.bashrc && nvm install 18.13 
# use npm command in sh
RUN apt install npm -y

COPY . /home/ros-server
WORKDIR /home/ros-server

RUN  npm install

CMD ["npm", "start"]