const deepCopy = (data: unknown) => JSON.parse(JSON.stringify(data));

export default deepCopy;
