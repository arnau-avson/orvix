type LogoProps = {
  className?: string
  title?: string
}

export function Logo({ className, title = 'Orvix' }: LogoProps) {
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      viewBox="0 0 156 40"
      fill="none"
      role="img"
      aria-label={title}
      className={className}
    >
      <title>{title}</title>
      <g transform="translate(4 4)">
        <ellipse
          cx="16"
          cy="16"
          rx="14.5"
          ry="5.5"
          stroke="currentColor"
          strokeWidth="2"
          transform="rotate(-28 16 16)"
        />
        <circle cx="16" cy="16" r="5.25" fill="currentColor" />
      </g>
      <text
        x="44"
        y="27.5"
        fontFamily="'Inter','SF Pro Display',-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,system-ui,sans-serif"
        fontSize="22"
        fontWeight="700"
        letterSpacing="-0.6"
        fill="currentColor"
      >
        orvix
      </text>
    </svg>
  )
}
