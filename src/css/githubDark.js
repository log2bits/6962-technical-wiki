const theme = {
    plain: {
        backgroundColor: "#161b22",
        color: "#C9D1D9",
    },
    styles: [
        {
            types: ["comment"],
            style: {
                color: "#8B949E",
            },
        },
        {
            types: ["boolean"],
            style: {
                color: "#FF7B72",
            },
        },
        {
            types: ["constant", "symbol", "url"],
            style: {
                color: "#79C0FF",
            },
        },
        {
            types: ["atrule","keyword"],
            style: {
                color: "#FFA657",
            },
        },
        {
            types: ["variable","function","entity","property-access"],
            style: {
                color: "#C9D1D9",
            },
        },
        {
            types: ["function-variable"],
            style: {
                color: "#D2A8FF",
            },
        },
        {
            types: ["tag"],
            style: {
                color: "#7EE787",
            },
        },
        {
            types: ["attr-name","attr-value"],
            style: {
                color: "#79C0FF",
            },
        },
        {
            types: ["number", "operator"],
            style: {
                color: "#A5D6FF",
            },
        },
        {
            types: ["string"],
            style: {
                color: "#A5D6FF",
            },
        },
        {
            types: ["invalid"],
            style: {
                color: "#FFA198",
                fontStyle: "italic",
            },
        },
    ],
};

export default theme;