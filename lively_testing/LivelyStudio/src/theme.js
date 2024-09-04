export const getTheme = (primaryColor) => ({
    name: "CoFrame",
    rounding: 4,
    defaultMode: "dark",
    global: {
      colors: {
        brand: primaryColor,
        // background: "#000000",
        control: primaryColor,
      },
      font: {
        family: "Helvetica",
      },
      focus: {
        border: {
          color: primaryColor,
        },
      },
      input: {
        padding: 4,
        extend: { backgroundColor: "#FFFFFF55" },
      },
      // edgeSize: {large: 50, small: 10, medium: 15}
    },
    button: {
      border: {
        radius: "4px",
      },
    },
    radioButton: {
      size: "16px",
      border: { color: "#00000088" },
    },
    checkBox: {
      size: "20px",
      border: { color: "#00000088" },
      color: primaryColor,
      hover: { border: { color: "#00000088" } },
    },
    textInput: {
      disabled: { opacity: 1 },
    },
    notification: {
      toast: {
        container: {
          elevation: "none",
        },
      },
      container: {
        border: { color: "lightgrey" },
        background: {
          color: "background-front",
        },
      },
    },
    tab: {
      active: {
        background: primaryColor,
        color: "dark-1",
      },
      background: "dark-3",
      border: undefined,
      color: "white",
      hover: {
        background: "#444444",
        color: "white",
      },
      margin: undefined,
      pad: {
        bottom: undefined,
        horizontal: "small",
      },
      extend: {
        borderRadius: 4,
        padding: 6,
      },
    },
    tabs: {
      gap: "medium",
      header: {
        extend: { padding: 10 },
      },
      panel: { padding: 10 },
      extend: { padding: 10 },
    },
  })