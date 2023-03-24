/* eslint-disable no-template-curly-in-string */
export const environment = {
    // eslint-disable-next-line quote-props
    BACKEND_IP: '${process.env["BACKEND_IP"]}',
    // eslint-disable-next-line quote-props
    IS_SIMULATION: '${process.env["IS_SIMULATION"]}'
};
