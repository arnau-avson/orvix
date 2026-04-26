// Header.tsx
const navItems = [
  { label: 'Producto', href: '#producto' },
  { label: 'Tecnología', href: '#tecnologia' },
  { label: 'Casos de uso', href: '#casos-de-uso' },
  { label: 'Contacto', href: '#contacto' },
]

export function Header() {
  return (
    <div className="absolute inset-x-0 top-4 z-50 mx-auto w-full max-w-5xl px-4">
      <nav className="flex items-center justify-between rounded-2xl border border-white/10 bg-white/80 px-6 py-3 backdrop-blur-md">
        {/* Logo */}
        <div className="text-lg font-semibold text-gray-900">
          Orvix
        </div>

        {/* Navigation Links */}
        <ul className="flex items-center gap-8">
          {navItems.map((item) => (
            <li key={item.label}>
              <a
                href={item.href}
                className="text-sm font-medium text-gray-600 transition-colors hover:text-gray-900"
              >
                {item.label}
              </a>
            </li>
          ))}
        </ul>

        {/* CTA Button */}
        <button className="rounded-full bg-gray-900 px-5 py-2 text-sm font-medium text-white transition-transform hover:scale-105 hover:bg-gray-800">
          Empezar
        </button>
      </nav>
    </div>
  )
}