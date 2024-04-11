const theme = {
    plain: {
        backgroundColor: "#161b22",
        color: "#C9D1D9",
    },
    styles: [
        {
			types: [
				"comment",
				"punctuation.definition.comment",
				"string.comment"
			],
			style: {
				color: "#8B949E"
			}
		},
		{
			types: [
				"constant.other.placeholder",
				"constant.character"
			],
			style: {
				color: "#FF7B72"
			}
		},
		{
			types: [
				"constant",
				"constant",
				"variable.other.constant",
				"variable.other.enummember",
				"variable.language",
				"entity"
			],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: [
				"entity.name",
				"meta.export.default",
				"meta.definition.variable"
			],
			style: {
				color: "#FFA657"
			}
		},
		{
			types: [
				"variable.parameter.function",
				"meta.jsx.children",
				"meta.block",
				"meta.tag.attributes",
				"constant",
				"meta.object.member",
				"meta.embedded.expression"
			],
			style: {
				color: "#E6EDF3"
			}
		},
		{
			types: ["function"],
			style: {
				color: "#D2A8FF"
			}
		},
		{
			types: [
				"tag",
				"support.class.component"
			],
			style: {
				color: "#7EE787"
			}
		},
		{
			types: ["keyword"],
			style: {
				color: "#FF7B72"
			}
		},
		{
			types: [
				"storage",
				"storage.type"
			],
			style: {
				color: "#FF7B72"
			}
		},
		{
			types: [
				"storage.modifier.package",
				"storage.modifier.import",
				"storage.type.java"
			],
			style: {
				color: "#E6EDF3"
			}
		},
		{
			types: [
				"string",
				"string punctuation.section.embedded source"
			],
			style: {
				color: "#A5D6FF"
			}
		},
		{
			types: ["support"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: ["meta.property-name"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: ["variable"],
			style: {
				color: "#FFA657"
			}
		},
		{
			types: ["variable.other"],
			style: {
				color: "#E6EDF3"
			}
		},
		{
			types: ["invalid.broken"],
			style: {
				color: "#FFA198",
				fontStyle: "italic"
			}
		},
		{
			types: ["invalid.deprecated"],
			style: {
				color: "#FFA198",
				fontStyle: "italic"
			}
		},
		{
			types: ["invalid.illegal"],
			style: {
				color: "#FFA198",
				fontStyle: "italic"
			}
		},
		{
			types: ["invalid.unimplemented"],
			style: {
				color: "#FFA198",
				fontStyle: "italic"
			}
		},
		{
			types: ["carriage-return"],
			style: {
				color: "#F0F6FC",
				"background": "#FF7B72",
				fontStyle: "italic underline"
			}
		},
		{
			types: ["message.error"],
			style: {
				color: "#FFA198"
			}
		},
		{
			types: ["string variable"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: [
				"source.regexp",
				"string.regexp"
			],
			style: {
				color: "#A5D6FF"
			}
		},
		{
			types: [
				"string.regexp.character-class",
				"string.regexp constant.character.escape",
				"string.regexp source.ruby.embedded",
				"string.regexp string.regexp.arbitrary-repitition"
			],
			style: {
				color: "#A5D6FF"
			}
		},
		{
			types: ["string.regexp constant.character.escape"],
			style: {
				color: "#7EE787",
				fontStyle: "bold"
			}
		},
		{
			types: ["support.constant"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: ["support.variable"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: ["support.type.property-name.json"],
			style: {
				color: "#7EE787"
			}
		},
		{
			types: ["meta.module-reference"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: ["punctuation.definition.list.begin.markdown"],
			style: {
				color: "#FFA657"
			}
		},
		{
			types: [
				"markup.heading",
				"markup.heading entity.name"
			],
			style: {
				color: "#79C0FF",
				fontStyle: "bold"
			}
		},
		{
			types: ["markup.quote"],
			style: {
				color: "#7EE787"
			}
		},
		{
			types: ["markup.italic"],
			style: {
				color: "#E6EDF3",
				fontStyle: "italic"
			}
		},
		{
			types: ["markup.bold"],
			style: {
				color: "#E6EDF3",
				fontStyle: "bold"
			}
		},
		{
			types: [
				"markup.underline"
			],
			style: {
				fontStyle: "underline"
			}
		},
		{
			types: [
				"markup.strikethrough"
			],
			style: {
				fontStyle: "strikethrough"
			}
		},
		{
			types: ["markup.inline.raw"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: [
				"markup.deleted",
				"meta.diff.header.from-file",
				"punctuation.definition.deleted"
			],
			style: {
				color: "#FFA198",
				"background": "#490202"
			}
		},
		{
			types: [
				"punctuation.section.embedded"
			],
			style: {
				color: "#FF7B72"
			}
		},
		{
			types: [
				"markup.inserted",
				"meta.diff.header.to-file",
				"punctuation.definition.inserted"
			],
			style: {
				color: "#7EE787",
				"background": "#04260F"
			}
		},
		{
			types: [
				"markup.changed",
				"punctuation.definition.changed"
			],
			style: {
				color: "#FFA657",
				"background": "#5A1E02"
			}
		},
		{
			types: [
				"markup.ignored",
				"markup.untracked"
			],
			style: {
				color: "#161B22",
				"background": "#79C0FF"
			}
		},
		{
			types: ["meta.diff.range"],
			style: {
				color: "#D2A8FF",
				fontStyle: "bold"
			}
		},
		{
			types: ["meta.diff.header"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: ["meta.separator"],
			style: {
				color: "#79C0FF",
				fontStyle: "bold"
			}
		},
		{
			types: ["meta.output"],
			style: {
				color: "#79C0FF"
			}
		},
		{
			types: [
				"brackethighlighter.tag",
				"brackethighlighter.curly",
				"brackethighlighter.round",
				"brackethighlighter.square",
				"brackethighlighter.angle",
				"brackethighlighter.quote"
			],
			style: {
				color: "#8B949E"
			}
		},
		{
			types: ["brackethighlighter.unmatched"],
			style: {
				color: "#FFA198"
			}
		},
		{
			types: [
				"constant.other.reference.link",
				"string.other.link"
			],
			style: {
				color: "#A5D6FF"
			}
		},
		{
			types: ["token.info-token"],
			style: {
				color: "#6796E6"
			}
		},
		{
			types: ["token.warn-token"],
			style: {
				color: "#CD9731"
			}
		},
		{
			types: ["token.error-token"],
			style: {
				color: "#F44747"
			}
		},
		{
			types: ["token.debug-token"],
			style: {
				color: "#B267E6"
			}
		}
        
    ],
};

export default theme;