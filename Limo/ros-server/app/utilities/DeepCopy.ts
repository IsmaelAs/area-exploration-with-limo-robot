const deepCopy = (data: unknown) => {
  if (!data) return data;

  return JSON.parse(JSON.stringify(data));
};

export default deepCopy;
