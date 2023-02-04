FROM ros:noetic

RUN apt update
RUN apt install nodejs && apt install npm

COPY . /home/ros-server
WORKDIR /home/ros-server

RUN npm install

CMD ["npm", "start"]