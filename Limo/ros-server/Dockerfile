FROM ros:noetic

RUN apt update 
RUN apt install nodejs && apt install npm

COPY . .
RUN npm install

CMD ["npm", "start"]