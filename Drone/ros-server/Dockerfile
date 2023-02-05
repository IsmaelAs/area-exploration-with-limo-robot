FROM ros:noetic

RUN apt update
RUN apt install nodejs -y && apt install npm -y

COPY . /home/ros-server
WORKDIR /home/ros-server

RUN npm install

CMD ["npm", "start"]